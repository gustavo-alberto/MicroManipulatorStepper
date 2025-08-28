
//*** CLASS *****************************************************************************

class Pose6DF;

//--- IKinemtaicModel -------------------------------------------------------------------

class IKinemtaicModel {
  public:
    virtual ~IKinemtaicModel() {};

    // returns the number of joints
    virtual int get_joint_count();

    // computes the end effector pose from joint positions
    virtual bool foreward(const float* joint_positions, Pose6DF& pose) = 0;

    // computes joint positions from an end effector pose
    virtual bool inverse(const Pose6DF& pose, float* joint_positions) = 0;
};
