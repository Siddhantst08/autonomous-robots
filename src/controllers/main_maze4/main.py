"""Controller entry point.

Wires the single-threaded layers together and runs the mission:

    MyRobot (hardware/pose/motion)
      + Perception (cameras & lidar interpretation)
      + Navigator  (plan/follow/replan with reactive safety)
      -> Mission   (explore -> localise both pillars -> blue -> yellow)

No Supervisor is used; only the permitted onboard sensors are read.
"""

from my_robot import MyRobot
from perception import Perception
from navigation import Navigator
from mission import Mission


def main():
    """Construct the layers and execute the full mission once."""
    robot = MyRobot()

    # Prime the pipeline: take a few steps so sensors and odometry settle
    # (odometry only initialises once the compass reports a valid heading).
    for _ in range(5):
        if robot.step() == -1:
            return

    perception = Perception(robot)
    navigator = Navigator(robot, perception)
    mission = Mission(robot, perception, navigator)

    success = mission.run()
    print(f"[main] mission finished, success={success}")

    # Hold position after finishing so the world does not error out.
    robot.stop_motor()
    while robot.step() != -1:
        pass


if __name__ == "__main__":
    main()
