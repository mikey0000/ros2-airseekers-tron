uint8 SHAPE_UNKNOWN = 0

# 根据区域形状自动时使用覆盖方式
uint8 SHAPE_AUTO = 1

# 使用 弓字形
uint8 SHAPE_BOW  = 2

# 螺旋形
uint8 SHAPE_SPIRAL  = 3
uint8 shape

# 如果为 bow 时，割草角度（弧度）是多少，充电桩下桩方向为0度
float32 bow_angle_radians

# 规划间距
float64 distance

# 是否启用AI推荐
bool ai_recommend

string polygon_uuid

# 如果x和y都为0时，表示没有断点
geometry_msgs/Pose break_pose

# use PlannerPath.type 定义的枚举
uint8 break_path_type

#割草顺序 1:Edge Moving 2:Working Area Moving 3:Restricted Area Edging 4:Patch 如1234,2134等
uint32 cut_sequence
