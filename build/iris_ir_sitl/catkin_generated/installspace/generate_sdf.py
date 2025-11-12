#!/usr/bin/env python2

import rospy
import os
import rospkg

def generate_sdf_for_uav(uav_id):
    """为特定无人机生成SDF文件"""
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('iris_ir_sitl')
    
    # SDF模板
    sdf_template = '''<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="iris_ir_{uav_id}">
    <!-- 包含标准iris模型 -->
    <include>
      <uri>model://iris</uri>
    </include>
    
    <!-- 添加红外发射器插件 -->
    <plugin name="infrared_emitter_plugin_{uav_id}" filename="libinfrared_emitter_plugin.so">
      <updateRate>10</updateRate>
      <range>15.0</range>
      <signalStrength>1.0</signalStrength>
    </plugin>

    <!-- 添加红外接收器插件 -->
    <plugin name="infrared_receiver_plugin_{uav_id}" filename="libinfrared_receiver_plugin.so">
      <updateRate>10</updateRate>
      <range>15.0</range>
      <sensitivity>1.0</sensitivity>
    </plugin>
  </model>
</sdf>'''
    
    # 填充模板
    sdf_content = sdf_template.format(uav_id=uav_id)
    
    # 确保模型目录存在
    model_dir = os.path.join(package_path, 'models/iris_zsj_ir')
    os.makedirs(model_dir, exist_ok=True)
    
    # 写入SDF文件
    sdf_file = os.path.join(model_dir, f'model_{uav_id}.sdf')
    with open(sdf_file, 'w') as f:
        f.write(sdf_content)
    
    rospy.loginfo("Generated SDF file for UAV %s: %s", uav_id, sdf_file)
    return sdf_file

if __name__ == '__main__':
    rospy.init_node('generate_sdf')
    
    # 从参数获取无人机ID
    uav_id = rospy.get_param('~uav_id', 1)
    
    try:
        generate_sdf_for_uav(uav_id)
        rospy.loginfo("SDF generation completed for UAV %s", uav_id)
    except Exception as e:
        rospy.logerr("Failed to generate SDF for UAV %s: %s", uav_id, str(e))