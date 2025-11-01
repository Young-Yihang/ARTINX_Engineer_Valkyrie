import numpy as np
from scipy.spatial.transform import Rotation

def main():
    """
    此脚本读取 link1 的原始物理属性，并根据一个固定的旋转（绕X轴90度），
    计算出新的、已经“烘焙”了旋转的物理属性。
    """
    print("--- 正在为 link1_double8009 计算修正后的物理属性 ---")

    # --- 1. 从您的 URDF 文件中复制的原始数据 ---
    # 这些是 link1 在其自身坐标系未旋转时的物理属性
    
    # 原始质心位置 (CoM)
    com_original = np.array([-0.0154999931131982, -0.0261644185287386, -0.00790164285412044])

    # 原始惯性张量参数
    ixx = 0.00126118311285525
    ixy = -9.13199961592521E-06
    ixz = -4.1718719896748E-07
    iyy = 0.00132467695230876
    iyz = -1.19448129551972E-06
    izz = 0.00217005077700242

    # 构建原始惯性张量矩阵
    I_original = np.array([
        [ixx, ixy, ixz],
        [ixy, iyy, iyz],
        [ixz, iyz, izz]
    ])

    # --- 2. 定义需要“烘焙”的旋转 ---
    # 这个旋转对应于您在 link1 的 <visual> 和 <inertial> 中添加的 rpy="1.5708 0 0"
    rotation_rad = 1.5708
    rotation_axis = 'x'
    rot = Rotation.from_euler(rotation_axis, rotation_rad, degrees=False)
    R = rot.as_matrix()

    # --- 3. 计算新属性 ---

    # 3.1 计算新的质心位置: CoM_new = R * CoM_original
    com_new = R @ com_original

    # 3.2 计算新的惯性张量: I_new = R * I_original * R^T
    I_new = R @ I_original @ R.transpose()

    # --- 4. 生成可直接复制的 URDF 代码块 ---
    print("\n✅ 计算完成！请用下面的代码块替换掉 link1_double8009 中现有的 <inertial> 块：")
    print("---------------------------------------------------------------------------------")
    print(f"""
    <inertial>
      <!-- 物理属性已根据 rpy="1.5708 0 0" 进行了数学变换 -->
      <origin
        xyz="{com_new[0]:.12f} {com_new[1]:.12f} {com_new[2]:.12f}"
        rpy="0 0 0" />
      <mass
        value="1.22525961510696" />
      <inertia
        ixx="{I_new[0, 0]:.12f}"
        ixy="{I_new[0, 1]:.12f}"
        ixz="{I_new[0, 2]:.12f}"
        iyy="{I_new[1, 1]:.12f}"
        iyz="{I_new[1, 2]:.12f}"
        izz="{I_new[2, 2]:.12f}" />
    </inertial>
    """)
    print("---------------------------------------------------------------------------------")

if __name__ == '__main__':
    main()