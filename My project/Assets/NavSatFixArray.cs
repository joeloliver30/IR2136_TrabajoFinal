using RosSharp.RosBridgeClient.MessageTypes.Sensor;

namespace RosSharp.RosBridgeClient.MessageTypes.Custom
{
    [System.Serializable]
    public class NavSatFixArray : Message
    {
        public const string RosMessageName = "pymavlink_ros/msg/NavSatFixArray"; // <-- nombre completo ROS2
        public NavSatFix[] points;

        public NavSatFixArray() { }

        public NavSatFixArray(NavSatFix[] trajectory)
        {
            points = trajectory;
        }
    }
}
