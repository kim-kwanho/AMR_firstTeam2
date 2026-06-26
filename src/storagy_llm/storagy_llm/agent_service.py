import rclpy
from rclpy.node import Node
from storagy_llm.robot_tools import ToolSet, create_tools
from storagy_interfaces.srv import Agent
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

import yaml
import cv2
import base64
import os
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from dotenv import load_dotenv
from langchain_openai import ChatOpenAI
from langchain.agents import create_agent
from langgraph.checkpoint.memory import MemorySaver


llm_dir = get_package_share_directory('storagy_llm')
# docker-compose env_file → 컨테이너 환경변수. 패키지 .env 는 빈 placeholder 일 수 있음.
env_file_path = Path(llm_dir) / '.env'
load_dotenv(dotenv_path=env_file_path, override=False)

for parent in Path(llm_dir).parents:
    workspace_env = parent / 'src' / '.env'
    if workspace_env.is_file():
        load_dotenv(dotenv_path=workspace_env, override=False)
        break

prompt_file = Path(llm_dir) / 'params/prompt.yaml'
with open(prompt_file, 'r', encoding='utf-8') as f:
    prompt_data = yaml.safe_load(f)    

class AgentLLM(Node):
    def __init__(self):
        super().__init__('agent_llm')

        self.srv = self.create_service(Agent, 'llm_agent', self.handle_question)

        api_key = os.getenv('OPENAI_API_KEY', '').strip()
        if not api_key:
            self.get_logger().error(
                'OPENAI_API_KEY 가 비어 있습니다. 프로젝트 루트 .env 에 키를 넣은 뒤 '
                '`docker compose up -d --force-recreate` 로 컨테이너를 다시 띄우세요. '
                '(restart 만으로는 env 가 갱신되지 않습니다.)'
            )
        self.llm = ChatOpenAI(model="gpt-4o-mini", temperature=0)
        
        self.bridge = CvBridge()
        self.latest_image_msg = None
        
        # 1. Subscribe to camera image
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

        yaml_file = Path(llm_dir) / 'params/points.yaml'
        with open(yaml_file, 'r') as f:
            config = yaml.safe_load(f)

        places = {name: (info["x"], info["y"], info["qz"], info["qw"]) for name, info in config["places"].items()}
        self.tool_set = ToolSet(places, explain_fn=self.explain_current_view)
        tool_list = create_tools(self.tool_set)

        # langchain v1.2+ 의 create_agent API 사용
        # checkpointer 를 지정하면 session 별 대화 히스토리를 자동 관리합니다.
        self.checkpointer = MemorySaver()
        self.agent_graph = create_agent(
            model=self.llm,
            tools=tool_list,
            system_prompt=prompt_data["system"],
            checkpointer=self.checkpointer,
        )
        
        self.get_logger().info("agent service start")

    def image_callback(self, msg: Image):
        self.latest_image_msg = msg

    def explain_current_view(self) -> str:
        if self.latest_image_msg is None:
            return "현재 전방 카메라 이미지 데이터를 수신하지 못했습니다. 카메라 토픽이 켜져 있는지 확인해 주세요."
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image_msg, desired_encoding='bgr8')
            _, buffer = cv2.imencode('.jpg', cv_image)
            img_base64 = base64.b64encode(buffer).decode('utf-8')
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return f"이미지 변환 중 오류가 발생했습니다: {e}"
        
        try:
            from langchain_core.messages import HumanMessage
            message = HumanMessage(
                content=[
                    {
                        "type": "text", 
                        "text": "로봇의 전방 카메라에 찍힌 사진입니다. 현재 앞에 무엇이 보이는지 한국어로 정중하고 친근하게 설명해주세요."
                    },
                    {
                        "type": "image_url",
                        "image_url": {"url": f"data:image/jpeg;base64,{img_base64}"},
                    },
                ]
            )
            response = self.llm.invoke([message])
            return response.content
        except Exception as e:
            self.get_logger().error(f"OpenAI call failed: {e}")
            return f"OpenAI API 호출 중 오류가 발생했습니다: {e}"

    def process_query(self, query):
        # session_id 기반으로 대화 히스토리를 유지합니다.
        config = {"configurable": {"thread_id": "storagy"}}
        result = self.agent_graph.invoke(
            {"messages": [{"role": "user", "content": query}]},
            config=config,
        )
        # 최종 AI 메시지를 반환
        messages = result.get("messages", [])
        if messages:
            return messages[-1].content
        return str(result)

    def handle_question(self, request, response):
        self.get_logger().info(f"💬: {request.question}"+"\n")
        if not os.getenv('OPENAI_API_KEY', '').strip():
            response.answer = (
                "OpenAI API 키가 설정되지 않았습니다. "
                "프로젝트 루트 `.env`에 `OPENAI_API_KEY=...`를 넣은 뒤 "
                "`docker compose up -d --force-recreate`로 컨테이너를 다시 시작해 주세요."
            )
            return response
        try:
            answer = self.process_query(request.question)
            response.answer = answer
        except Exception as e:
            self.get_logger().error(f"LLM query failed: {e}")
            err = str(e).lower()
            if 'api_key' in err or 'authentication' in err or 'incorrect api key' in err:
                response.answer = (
                    "OpenAI API 키가 유효하지 않거나 인증에 실패했습니다. "
                    "`.env`의 키를 확인한 뒤 `docker compose up -d --force-recreate`를 실행해 주세요."
                )
            else:
                response.answer = f"처리 중 오류가 발생했습니다: {e}"
        return response
    
def main(args=None):
    rclpy.init(args=args)
    agent = AgentLLM()
    try:
        rclpy.spin(agent) 
    finally:
        agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
