enum TrajectoryType
{
    PTP,
    LIN,
    CIRC,
    BEZ
};


class Trajectory
{
    public:
    Trajectory()
    {

    }

    TrajectoryType getType()
    {
        return type;
    
    ;}

    double totalTime()
    {
        return total_time;
    };

    private:
    TrajectoryType type = PTP;
    double total_time = 0;
    double 



}