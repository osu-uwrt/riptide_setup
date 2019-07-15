import actionlib
import riptide_controllers.msg


def performActions(*actions):
    for a in actions:
        a.wait_for_result()

def depthAction(depth):
    """ 
    Travel to and hold depth
  
    Parameters: 
        depth (float): The depth in meters to travel to.
    """
    client = actionlib.SimpleActionClient(
        "go_to_depth", riptide_controllers.msg.GoToDepthAction)
    client.wait_for_server()

    client.send_goal(riptide_controllers.msg.GoToDepthGoal(depth))
    return client

def rollAction(angle):
    """ 
    Travel to and hold roll
  
    Parameters: 
        angle (float): The roll in degrees to travel to.
    """
    client = actionlib.SimpleActionClient(
        "go_to_roll", riptide_controllers.msg.GoToRollAction)
    client.wait_for_server()

    client.send_goal(riptide_controllers.msg.GoToRollGoal(angle))
    return client

def pitchAction(angle):
    """ 
    Travel to and hold pitch
  
    Parameters: 
        angle (float): The pitch in degrees to travel to.
    """
    client = actionlib.SimpleActionClient(
        "go_to_pitch", riptide_controllers.msg.GoToPitchAction)
    client.wait_for_server()

    client.send_goal(riptide_controllers.msg.GoToPitchGoal(angle))
    return client

def yawAction(angle):
    """ 
    Travel to and hold yaw
  
    Parameters: 
        angle (float): The yaw in degrees to travel to.
    """
    client = actionlib.SimpleActionClient(
        "go_to_yaw", riptide_controllers.msg.GoToYawAction)
    client.wait_for_server()

    client.send_goal(riptide_controllers.msg.GoToYawGoal(angle))
    return client

def moveAction(x, y):
    """ 
    Move a distance while upright
  
    Parameters: 
        x (float): The x distance to travel in meters. This is in robot frame while upright
        y (float): The y distance to travel in meters. This is in robot frame while upright
    """
    client = actionlib.SimpleActionClient(
        "move_distance", riptide_controllers.msg.MoveDistanceAction)
    client.wait_for_server()

    client.send_goal(riptide_controllers.msg.MoveDistanceActionGoal(x, y))
    return client

def waitAction(obj, times):
    """ 
    Action completes when object seen "times" times
  
    Parameters: 
        obj (str): The object to look for
        times (int): The number of times to see the object before continuing
    """
    client = actionlib.SimpleActionClient(
        "wait", riptide_controllers.msg.WaitAction)
    client.wait_for_server()

    client.send_goal(riptide_controllers.msg.WaitActionGoal(obj, times))
    return client

def gateManeuverAction():
    """Unimplemented"""
    pass

def alignAction(object):
    """Unimplemented"""
    pass

def gateTask(isLeft):
    """
    The gate task
    
    Parameters: 
        isLeft (bool): Whether the small side of the gate is on the left
    """
    client = actionlib.SimpleActionClient(
        "gate_task", riptide_controllers.msg.GateTaskAction)
    client.wait_for_server()

    client.send_goal(riptide_controllers.msg.GateTaskActionGoal())
    return client