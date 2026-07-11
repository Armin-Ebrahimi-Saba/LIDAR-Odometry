import sys
import os

path = '/root/workspace/src/LIO_SAM_6AXIS/LIO-SAM-6AXIS/config/indoor_ouster128.yaml'

if not os.path.exists(path):
    print(f"File {path} not found.")
    sys.exit(0)

with open(path, 'r') as f:
    content = f.read()

content = content.replace('N_SCAN: 128', 'N_SCAN: 32')
content = content.replace('imuFrequence: 200', 'imuFrequence: 100')


content = content.replace('imuAccNoise: 0.0011501915187049582', 'imuAccNoise: 0.01')
content = content.replace('imuGyrNoise: 5.084312924828687e-05', 'imuGyrNoise: 0.001')
content = content.replace('imuAccBiasN: 3.729854069064516e-05', 'imuAccBiasN: 0.0001')
content = content.replace('imuGyrBiasN: 6.509823412408377e-07', 'imuGyrBiasN: 0.00001')


trans_old = 'extrinsicTrans: [ -0.047781, 0.007303, -0.026583 ]'
trans_new = 'extrinsicTrans: [ 0.0, 0.0, 0.0 ]'
content = content.replace(trans_old, trans_new)

rot_old = '''extrinsicRot: [ 0.9999872, -0.0010636, -0.0049547,
                  0.0010324,  0.9999796, -0.0062985,
                  0.0049613,  0.0062933,  0.9999679 ]'''
rot_new = '''extrinsicRot: [ 1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0 ]'''
content = content.replace(rot_old, rot_new)

rpy_old = '''extrinsicRPY: [ 0.9999872, -0.0010636, -0.0049547,
                  0.0010324,  0.9999796, -0.0062985,
                  0.0049613,  0.0062933,  0.9999679 ]'''
rpy_new = '''extrinsicRPY: [ 1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0 ]'''
content = content.replace(rpy_old, rpy_new)

content = content.replace('imuGravity: 9.80511', 'imuGravity: -9.80511')

with open(path, 'w') as f:
    f.write(content)

print("Successfully patched params_ouster.yaml for Ouster OS0-32 internal IMU.")
