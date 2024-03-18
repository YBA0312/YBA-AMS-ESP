# YBA-AMS-ESP
需要使用ESP配网APP

先用着，有空再完善

| 电机引脚 | 对应IO |
| -------- | ------ |
| M1_UP    | 0      |
| M1_DOWN  | 1      |
| M2_UP    | 18     |
| M2_DOWN  | 19     |
| M3_UP    | 2      |
| M3_DOWN  | 3      |
| M4_UP    | 10     |
| M4_DOWN  | 6      |
| SW1U     | 7      |
| SW2U     | 5      |
| SW3U     | 4      |
| SW4U     | 8      |

TCP 通讯协议
端口 3333

| head   | dest_addr | src_addr | cmd  | len  | 电机位号 | 方向 |
| ------ | --------- | -------- | ---- | ---- | -------- | ---- |
| 0x2F2F | 0xFF      | 0xFE     | 0x01 | 0x02 | 0x00     | 0x01 |

0停止 1进料 2退料