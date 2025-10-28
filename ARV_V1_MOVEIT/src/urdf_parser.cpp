#include <string>
#include <vector>
#include <map>
#include <iostream>
#include <Eigen/Dense>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

class URDFDynamicsParser {
public:
    struct LinkDynamics {
        std::string name;
        double mass;                    // 质量 [kg]
        Eigen::Vector3d com;            // 质心位置 [m]
        Eigen::Matrix3d inertia;        // 惯性张量 [kg·m²]
    };
    
    struct JointInfo {
        std::string name;
        std::string type;               // revolute, prismatic, etc.
        Eigen::Vector3d axis;           // 关节轴方向
        double effort_limit;            // 力矩限制 [N·m]
        double velocity_limit;          // 速度限制 [rad/s]
        double lower_limit;             // 位置下限 [rad]
        double upper_limit;             // 位置上限 [rad]
        double damping;                 // 阻尼系数
        double friction;                // 摩擦系数
    };
    
    // 核心功能：解析URDF文件
    bool parseURDF(const std::string& urdf_path);
    
    // 核心功能：从URDF字符串解析
    bool parseURDFString(const std::string& urdf_string);
    
    // 获取KDL运动链（最重要！）
    KDL::Chain getKDLChain() const { return kdl_chain_; }
    
    // 获取有序的连杆/关节列表
    std::vector<LinkDynamics> getLinkDynamics() const { return links_ordered_; }
    std::vector<JointInfo> getJointInfo() const { return joints_ordered_; }
    
    // 打印提取的信息
    void printSummary() const;

private:
    KDL::Chain kdl_chain_;                      // KDL运动链
    std::vector<LinkDynamics> links_ordered_;   // 按顺序的连杆
    std::vector<JointInfo> joints_ordered_;     // 按顺序的关节
    
    // 辅助函数：从URDF模型提取数据
    void extractFromModel(const urdf::Model& model);
};

// ============= 实现部分 =============

bool URDFDynamicsParser::parseURDF(const std::string& urdf_path) {
    // 从文件加载URDF
    urdf::Model model;
    if (!model.initFile(urdf_path)) {
        std::cerr << "❌ Failed to parse URDF file: " << urdf_path << std::endl;
        return false;
    }
    
    std::cout << "✅ Successfully loaded URDF: " << model.getName() << std::endl;
    extractFromModel(model);
    
    // 从文件构建KDL chain
    KDL::Tree tree;
    if (!kdl_parser::treeFromFile(urdf_path, tree)) {
        std::cerr << "❌ Failed to build KDL tree from URDF" << std::endl;
        return false;
    }
    
    // 提取运动链: base_link → link6_2006roll
    if (!tree.getChain("base_link", "link6_2006roll", kdl_chain_)) {
        std::cerr << "❌ Failed to extract KDL chain" << std::endl;
        return false;
    }
    
    std::cout << "✅ KDL chain extracted: " 
              << kdl_chain_.getNrOfSegments() << " segments, "
              << kdl_chain_.getNrOfJoints() << " joints" << std::endl;
    
    return true;
}

bool URDFDynamicsParser::parseURDFString(const std::string& urdf_string) {
    // 从字符串加载URDF
    urdf::Model model;
    if (!model.initString(urdf_string)) {
        std::cerr << "❌ Failed to parse URDF string" << std::endl;
        return false;
    }
    
    std::cout << "✅ Successfully parsed URDF: " << model.getName() << std::endl;
    extractFromModel(model);
    
    // 从字符串构建KDL chain
    KDL::Tree tree;
    if (!kdl_parser::treeFromString(urdf_string, tree)) {
        std::cerr << "❌ Failed to build KDL tree" << std::endl;
        return false;
    }
    
    // 提取运动链
    if (!tree.getChain("base_link", "link6_2006roll", kdl_chain_)) {
        std::cerr << "❌ Failed to extract KDL chain" << std::endl;
        return false;
    }
    
    std::cout << "✅ KDL chain extracted: " 
              << kdl_chain_.getNrOfSegments() << " segments, "
              << kdl_chain_.getNrOfJoints() << " joints" << std::endl;
    
    return true;
}

void URDFDynamicsParser::extractFromModel(const urdf::Model& model) {
    links_ordered_.clear();
    joints_ordered_.clear();
    
    // 定义你的机械臂的关节顺序
    std::vector<std::string> joint_names = {
        "joint_1", "joint_2", "joint_3", 
        "joint_4", "joint_5", "joint_6"
    };
    
    // 按顺序提取关节信息
    for (const auto& joint_name : joint_names) {
        auto joint_ptr = model.getJoint(joint_name);
        if (!joint_ptr) {
            std::cerr << "⚠️  Joint not found: " << joint_name << std::endl;
            continue;
        }
        
        JointInfo ji;
        ji.name = joint_name;
        ji.type = (joint_ptr->type == urdf::Joint::REVOLUTE) ? "revolute" : "other";
        
        // 关节轴
        ji.axis << joint_ptr->axis.x, joint_ptr->axis.y, joint_ptr->axis.z;
        
        // 关节限制
        if (joint_ptr->limits) {
            ji.effort_limit = joint_ptr->limits->effort;
            ji.velocity_limit = joint_ptr->limits->velocity;
            ji.lower_limit = joint_ptr->limits->lower;
            ji.upper_limit = joint_ptr->limits->upper;
        } else {
            ji.effort_limit = 0.0;
            ji.velocity_limit = 0.0;
            ji.lower_limit = 0.0;
            ji.upper_limit = 0.0;
        }
        
        // 动力学参数
        if (joint_ptr->dynamics) {
            ji.damping = joint_ptr->dynamics->damping;
            ji.friction = joint_ptr->dynamics->friction;
        } else {
            ji.damping = 0.0;
            ji.friction = 0.0;
        }
        
        joints_ordered_.push_back(ji);
        
        // 提取对应的连杆信息
        auto link_ptr = model.getLink(joint_ptr->child_link_name);
        if (link_ptr && link_ptr->inertial) {
            LinkDynamics ld;
            ld.name = link_ptr->name;
            ld.mass = link_ptr->inertial->mass;
            
            // 质心位置
            ld.com << link_ptr->inertial->origin.position.x,
                      link_ptr->inertial->origin.position.y,
                      link_ptr->inertial->origin.position.z;
            
            // 惯性张量（对称矩阵）
            ld.inertia << link_ptr->inertial->ixx, link_ptr->inertial->ixy, link_ptr->inertial->ixz,
                          link_ptr->inertial->ixy, link_ptr->inertial->iyy, link_ptr->inertial->iyz,
                          link_ptr->inertial->ixz, link_ptr->inertial->iyz, link_ptr->inertial->izz;
            
            links_ordered_.push_back(ld);
        }
    }
    
    std::cout << "📊 Extracted " << joints_ordered_.size() << " joints and " 
              << links_ordered_.size() << " links" << std::endl;
}

void URDFDynamicsParser::printSummary() const {
    std::cout << "\n========== URDF 运动链提取摘要 ==========" << std::endl;
    
    std::cout << "\n【KDL运动链】" << std::endl;
    std::cout << "  Segments: " << kdl_chain_.getNrOfSegments() << std::endl;
    std::cout << "  Joints:   " << kdl_chain_.getNrOfJoints() << std::endl;
    
    std::cout << "\n【关节信息】(" << joints_ordered_.size() << " 个)" << std::endl;
    for (size_t i = 0; i < joints_ordered_.size(); i++) {
        const auto& ji = joints_ordered_[i];
        std::cout << "  [" << i+1 << "] " << ji.name 
                  << " | 类型: " << ji.type
                  << " | 轴: (" << ji.axis.transpose() << ")"
                  << " | 力矩限制: " << ji.effort_limit << " N·m"
                  << " | 阻尼: " << ji.damping << std::endl;
    }
    
    std::cout << "\n【连杆动力学】(" << links_ordered_.size() << " 个)" << std::endl;
    for (size_t i = 0; i < links_ordered_.size(); i++) {
        const auto& ld = links_ordered_[i];
        std::cout << "  [" << i+1 << "] " << ld.name 
                  << " | 质量: " << ld.mass << " kg"
                  << " | COM: (" << ld.com.transpose() << ") m" << std::endl;
    }
    
    std::cout << "\n========================================\n" << std::endl;
}

// ============= 测试主函数 (可选) =============
#ifdef TEST_URDF_PARSER
int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "用法: " << argv[0] << " <URDF文件路径>" << std::endl;
        return 1;
    }
    
    URDFDynamicsParser parser;
    
    if (!parser.parseURDF(argv[1])) {
        std::cerr << "解析失败!" << std::endl;
        return 1;
    }
    
    parser.printSummary();
    
    // 测试获取KDL chain
    KDL::Chain chain = parser.getKDLChain();
    std::cout << "✅ 成功获取KDL运动链，可用于动力学计算！" << std::endl;
    
    return 0;
}
#endif



