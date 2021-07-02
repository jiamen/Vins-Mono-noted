camera_model
    
    calib
        CameraCalibration.c 封装了各个相机模型
    camera_models
        camera.c 构造camera类，为各种相机模型提供虚函数接口
        PinholeCamera.c 针孔相机模型
        ScaramuzzaCamera.c SCARAMUZZA相机模型
        CataCamera.c MEI相机模型
        EquidistantCamera.c KANNALA_BRANDT相机模型
        CostFunctionFactory.c 代价函数，用于优化相机模型中的参数
    chessboard 用于检测棋盘格特征点
    gpl 经纬度变换（好像并没用调用）
    
    sparse_graph
        Transform.c 实现数据变量的操作

    intrinsic_calib.cc 相机矫正模块主函数，提供了人机交互接口
