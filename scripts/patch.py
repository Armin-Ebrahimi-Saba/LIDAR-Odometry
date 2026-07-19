import os
import glob
import re

if os.path.exists('/workspace/src/LIO-SAM-6AXIS-src'):
    print("Found local LIO-SAM-6AXIS-src directory, copying to ROS workspace...")
    os.system('cp -r /workspace/src/LIO-SAM-6AXIS-src/* /root/workspace/src/LIO_SAM_6AXIS/LIO-SAM-6AXIS/src/')

def patch_file(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # Add EIGEN_MAKE_ALIGNED_OPERATOR_NEW to ParamServer, mapOptimization, IMUPreintegration, etc.
    classes_to_patch = ['ParamServer', 'mapOptimization', 'IMUPreintegration', 'FeatureExtraction', 'ImageProjection', 'TransformFusion', 'DataSaver']
    for cls in classes_to_patch:
        pattern = r"(class\s+" + cls + r"[\s\S]*?public:)"
        replacement = r"\1\n        EIGEN_MAKE_ALIGNED_OPERATOR_NEW\n"
        content = re.sub(pattern, replacement, content)

    # Patch std::make_unique that ignores EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    content = content.replace("std::make_unique<DataSaver>(saveDirectory, sequence);",
                              "std::unique_ptr<DataSaver>(new DataSaver(saveDirectory, sequence));")

    # Patch vector<gtsam::Pose3>
    content = content.replace("vector<gtsam::Pose3> loopPoseQueue;", 
                              "vector<gtsam::Pose3, Eigen::aligned_allocator<gtsam::Pose3>> loopPoseQueue;")
    content = content.replace("std::vector<gtsam::Pose3>", 
                              "std::vector<gtsam::Pose3, Eigen::aligned_allocator<gtsam::Pose3>>")
    # Patch main functions to heap-allocate classes
    content = content.replace("mapOptimization MO;", "mapOptimization* MO = new mapOptimization();")
    content = content.replace("&MO", "MO")
    
    content = content.replace("IMUPreintegration ImuP;", "IMUPreintegration* ImuP = new IMUPreintegration();")
    content = content.replace("TransformFusion TF;", "TransformFusion* TF = new TransformFusion();")
    content = content.replace("ImageProjection IP;", "ImageProjection* IP = new ImageProjection();")
    content = content.replace("FeatureExtraction FE;", "FeatureExtraction* FE = new FeatureExtraction();")
    content = content.replace("mapOptimizationGps MOG;", "mapOptimizationGps* MOG = new mapOptimizationGps();")
    
    # Patch OusterPointXYZIRT to match fix_ouster_bag.py EXACTLY
    old_struct = '''struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
//  uint32_t time;
    uint16_t reflectivity;
    uint8_t ring;
    std::uint16_t ambient;  // additional property of p.ouster
    float time;
    uint16_t noise;
    uint32_t range;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                          (uint16_t, reflectivity, reflectivity)
                                          (uint8_t, ring, ring)
                                          (std::uint16_t, ambient, ambient)
                                          (float, time, time)
                                          (uint16_t, noise, noise)
                                          (uint32_t, range, range)
)'''
    new_struct = '''struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                  (uint32_t, t, t)
                                  (uint16_t, ring, ring)
)'''
    content = content.replace(old_struct, new_struct)
    content = content.replace("dst.time = src.time;", "dst.time = src.t * 1e-9f;")
    
    with open(filepath, 'w') as f:
        f.write(content)

def main():
    files = glob.glob('src/*.cpp') + glob.glob('src/*.h') + glob.glob('include/*.h')
    for f in files:
        if os.path.isfile(f):
            patch_file(f)

if __name__ == "__main__":
    main()
