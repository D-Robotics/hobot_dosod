import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from collections import Counter

from ai_msgs.msg import PerceptionTargets
from std_msgs.msg import String

class Object2TextNode(Node):
    def __init__(self):
        super().__init__('object_to_text_node')
        self.get_logger().warn('Object To Text Node has been started.')


        # active: 表示是否处于“等待并处理DOSOD结果”的状态
        # detection_count: 已经收集到的DOSOD检测结果条数
        # obj_counter: 用于记录各类别检测到的最大数量
        self.active = False
        self.detection_count = 0
        self.obj_counter = Counter()

        # 这个变量手动设置：表示在收到一次ASR唤醒后，需要等待多少次DOSOD检测结果再统一生成句子
        self.num_detection_results_needed = 5

        # ============ 2) 中英类别映射表 =============
        # self.category_map = {
        #     "person": "人",
        #     "bicycle": "自行车",
        #     "car": "汽车",
        #     "motorcycle": "摩托车",
        #     "airplane": "飞机",
        #     "bus": "巴士",
        #     "train": "火车",
        #     "truck": "卡车",
        #     "boat": "船",
        #     "traffic light": "红绿灯",
        #     "fire hydrant": "消防栓",
        #     "stop sign": "停车标志",
        #     "parking meter": "停车计时器",
        #     "bench": "长凳",
        #     "bird": "鸟",
        #     "cat": "猫",
        #     "dog": "狗",
        #     "horse": "马",
        #     "sheep": "羊",
        #     "cow": "牛",
        #     "elephant": "大象",
        #     "bear": "熊",
        #     "zebra": "斑马",
        #     "giraffe": "长颈鹿",
        #     "backpack": "背包",
        #     "umbrella": "雨伞",
        #     "handbag": "手提包",
        #     "tie": "领带",
        #     "suitcase": "行李箱",
        #     "frisbee": "飞盘",
        #     "skis": "滑雪板",
        #     "snowboard": "单板滑雪板",
        #     "sports ball": "球",
        #     "kite": "风筝",
        #     "baseball bat": "棒球棒",
        #     "baseball glove": "棒球手套",
        #     "skateboard": "滑板",
        #     "surfboard": "冲浪板",
        #     "tennis racket": "网球拍",
        #     "bottle": "瓶子",
        #     "wine glass": "酒杯",
        #     "cup": "杯子",
        #     "fork": "叉子",
        #     "knife": "刀",
        #     "spoon": "勺子",
        #     "bowl": "碗",
        #     "banana": "香蕉",
        #     "apple": "苹果",
        #     "sandwich": "三明治",
        #     "orange": "橘子",
        #     "broccoli": "西兰花",
        #     "carrot": "胡萝卜",
        #     "hot dog": "热狗",
        #     "pizza": "披萨",
        #     "donut": "甜甜圈",
        #     "cake": "蛋糕",
        #     "chair": "椅子",
        #     "couch": "沙发",
        #     "potted plant": "盆栽",
        #     "bed": "床",
        #     "dining table": "餐桌",
        #     "toilet": "马桶",
        #     "tv": "电视",
        #     "laptop": "笔记本电脑",
        #     "mouse": "鼠标",
        #     "remote": "遥控器",
        #     "keyboard": "键盘",
        #     "cell phone": "手机",
        #     "microwave": "微波炉",
        #     "oven": "烤箱",
        #     "toaster": "烤面包机",
        #     "sink": "水槽",
        #     "refrigerator": "冰箱",
        #     "book": "书",
        #     "clock": "时钟",
        #     "vase": "花瓶",
        #     "scissors": "剪刀",
        #     "teddy bear": "泰迪熊",
        #     "hair drier": "吹风机",
        #     "toothbrush": "牙刷"
        # }
        self.category_map = {
            "desk": "办公桌",
            "chair": "椅子",
            "conference table": "会议桌",
            "whiteboard": "白板",
            "cabinet": "柜子",
            "projector": "投影仪",
            "laptop": "笔记本电脑",
            "monitor": "显示器",
            "keyboard": "键盘",
            "mouse": "鼠标",
            "microphone": "麦克风",
            "webcam": "网络摄像头",
            "router": "路由器",
            "speaker": "扬声器",
            "remote control": "遥控器",
            "mobile phone": "手机",
            "tablet": "平板电脑",
            "notebook": "笔记本",
            "pen": "钢笔",
            "marker pen": "马克笔",
            "highlighter": "荧光笔",
            "eraser": "橡皮擦",
            "sticky notes": "便利贴",
            "stapler": "订书机",
            "scissors": "剪刀",
            "tape dispenser": "胶带座",
            "file folder": "文件夹",
            "clipboard": "写字板",
            "bottle": "瓶子",
            "cup": "杯子",
            "mug": "马克杯",
            "water dispenser": "饮水机",
            "snack": "零食",
            "plate": "盘子",
            "fork": "叉子",
            "tissue box": "纸巾盒",
            "person": "人",
            "robot": "机器人",
            "packaging box": "包装盒",
            "box": "盒子"
        }
        # ============ 3) 订阅 /asr_text 以及 /hobot_dosod =============
        # 当ASR有新的文本时，开启一次“等待处理DOSOD结果”的流程
        self.subscription_asr = self.create_subscription(
            String,
            '/asr_text',
            self.asr_callback,
            10
        )

        # 持续接受DOSOD检测结果，但只有当active==True时我们才真正处理
        self.subscription_dosod = self.create_subscription(
            PerceptionTargets,
            '/hobot_dosod',
            self.listener_callback,
            10
        )

        # ============ 4) 负责发布最终中文句子 =============
        self.publisher = self.create_publisher(String, '/tts_text', 10)


    def asr_callback(self, msg: String):
        """
        当 /asr_text 有新文本消息时调用：
        1) active设为True, 表示接下来需要处理DOSOD检测结果。
        2) 重置detection_count和obj_counter。
        """
        self.get_logger().info(f'收到新的ASR文本: "{msg.data}"，即将开始收集DOSOD检测结果。')

        self.active = True
        self.detection_count = 0
        self.obj_counter.clear()

    def listener_callback(self, msg: PerceptionTargets):
        """
        当 /hobot_dosod 有新的检测结果时调用。
        仅当active==True时，才会把检测结果纳入计数。
        如果收集到的检测结果条数达到num_detection_results_needed时，就生成最终句子并发布。
        """
        if not self.active:
            return

        # =========== 1. 统计本次检测到的类别及数量 ===========
        class_list = [target.type for target in msg.targets]
        current_counts = Counter(class_list)

        # =========== 2. 如果什么都没检测到怎么办？ ===========
        #   由于我们要合并多次检测的“最大值”，单次检测为空并不一定最后也为空；
        #   只有在收集完毕后若总的obj_counter仍为空才生成“我什么都没有检测到呀”。

        # =========== 3. 若发现某些类别数量 >= 1000，直接报错 ===========
        for obj_type, c in current_counts.items():
            if c >= 1000:
                self.publish_result("错误！检测到的物体数量大于1000")
                # 终止本次处理，重置状态
                self.active = False
                return

        # =========== 4. 将本次检测结果与历史记录比较，更新最大值 =============
        for obj_type, c in current_counts.items():
            # 保持同一类别的最大值
            if c > self.obj_counter[obj_type]:
                self.obj_counter[obj_type] = c

        # =========== 5. 累加处理次数，如果达到阈值，就生成最终句子 ===========
        self.detection_count += 1
        if self.detection_count >= self.num_detection_results_needed:
            # 说明已经收集到足够次的检测结果，生成句子
            self.generate_and_publish_sentence()
            # 生成完毕后，重置状态，表示本轮结束
            self.active = False

    def generate_and_publish_sentence(self):
        """
        将 obj_counter 中的最大检测结果拼成一段中文句子发布。
        """
        # 如果统计结果是空，直接输出“没检测到”
        if len(self.obj_counter) == 0:
            self.publish_result("我什么都没有检测到呀！")
            return

        # 拼装句子，示例格式：
        # “我好像看到了三个人，还有一个泰迪熊，还有一本书哦！”
        sentence_parts = []
        for i, (obj_type, count) in enumerate(self.obj_counter.items()):
            # 翻译成中文类别
            ch_type = self.category_map.get(obj_type, obj_type)  # 若没翻译就用原名

            # 转换数字 => 中文数词（<1000）
            count_str = self.number_to_chinese(count)

            part_text = f"{count_str}个{ch_type}"
            sentence_parts.append(part_text)

        if len(sentence_parts) == 1:
            # 只有一种物体
            final_sentence = f"我看到了 {sentence_parts[0]} 哇！"
        else:
            # 多种物体，用"、"做分隔
            middle_text = "、".join(sentence_parts[:-1])  # 前面拼在一起
            final_sentence = f"我看到 {middle_text}，还有 {sentence_parts[-1]}哇！"

        self.publish_result(final_sentence)

    def publish_result(self, text: str):
        """
        辅助函数：向 /tts_text 话题发布文本。
        """
        msg = String()
        msg.data = text
        self.publisher.publish(msg)
        self.get_logger().info(f'已发布结果: {text}')

    def number_to_chinese(self, num: int) -> str:
        """
        将数字(0~999) 转换成中文数词表示。
        如果 num >= 1000，直接返回空字符串（或其他提示），
        但实际上我们在 listener_callback 里已拦截过 >=1000 的情况。
        """
        if num < 0:
            return "零"  # 一般不会出现负数检测
        if num >= 1000:
            # 在listener_callback已经做拦截，这里只是兜底处理
            return ""

        # 下面是一个简易数词转换逻辑，处理范围 0~999
        ones = ["零", "一", "两", "三", "四", "五", "六", "七", "八", "九"]
        teens = ["十", "十一", "十二", "十三", "十四", "十五",
                 "十六", "十七", "十八", "十九"]
        tens = ["", "", "二十", "三十", "四十", "五十",
                "六十", "七十", "八十", "九十"]

        def two_digits(n):
            if n < 10:
                return ones[n]
            elif n < 20:
                return teens[n - 10]
            else:
                t = tens[n // 10]
                r = n % 10
                if r != 0:
                    t += ones[r]
                return t

        def three_digits(n):
            # 0 ~ 999
            if n < 100:
                return two_digits(n)
            else:
                hundreds_part = ones[n // 100] + "百"
                remainder = n % 100
                if remainder == 0:
                    return hundreds_part
                else:
                    return hundreds_part + two_digits(remainder)

        return three_digits(num)

def main(args=None):
    rclpy.init(args=args)
    node = Object2TextNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
