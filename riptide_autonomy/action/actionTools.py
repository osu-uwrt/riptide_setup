import actionlib
import riptide_controllers.msg
import riptide_autonomy.msg

lastActions = []

def checkPreempted(as_):
    global lastActions
    if as_.is_preempt_requested():
        for c in lastActions:
            c.cancel_all_goals()
        lastActions = []
       

def performActions(*actions):
    global lastActions
    lastActions = actions
    for a in actions:
        a.wait_for_result()

def getResult(action):
    action.wait_for_result()
    return action.get_result()

def depthAction(depth):
    """ 
    Travel to and hold depth
  
    Parameters: 
        depth (float): The depth in meters to travel to.
    """
    global lastActions
    client = actionlib.SimpleActionClient(
        "go_to_depth", riptide_controllers.msg.GoToDepthAction)
    client.wait_for_server()

    client.send_goal(riptide_controllers.msg.GoToDepthGoal(depth))
    lastActions = [client]
    return client

def rollAction(angle):
    """ 
    Travel to and hold roll
  
    Parameters: 
        angle (float): The roll in degrees to travel to.
    """
    global lastActions
    client = actionlib.SimpleActionClient(
        "go_to_roll", riptide_controllers.msg.GoToRollAction)
    client.wait_for_server()

    client.send_goal(riptide_controllers.msg.GoToRollGoal(angle))
    lastActions = [client]
    return client

def pitchAction(angle):
    """ 
    Travel to and hold pitch
  
    Parameters: 
        angle (float): The pitch in degrees to travel to.
    """
    global lastActions
    client = actionlib.SimpleActionClient(
        "go_to_pitch", riptide_controllers.msg.GoToPitchAction)
    client.wait_for_server()

    client.send_goal(riptide_controllers.msg.GoToPitchGoal(angle))
    lastActions = [client]
    return client

def yawAction(angle):
    """ 
    Travel to and hold yaw
  
    Parameters: 
        angle (float): The yaw in degrees to travel to.
    """
    global lastActions
    client = actionlib.SimpleActionClient(
        "go_to_yaw", riptide_controllers.msg.GoToYawAction)
    client.wait_for_server()

    client.send_goal(riptide_controllers.msg.GoToYawGoal(angle))
    lastActions = [client]
    return client

def moveAction(x, y):
    """ 
    Move a distance while upright
  
    Parameters: 
        x (float): The x distance to travel in meters. This is in robot frame while upright
        y (float): The y distance to travel in meters. This is in robot frame while upright
    """
    global lastActions
    client = actionlib.SimpleActionClient(
        "move_distance", riptide_controllers.msg.MoveDistanceAction)
    client.wait_for_server()

    client.send_goal(riptide_controllers.msg.MoveDistanceGoal(x, y))
    lastActions = [client]
    return client

def waitAction(obj, times):
    """ 
    Action completes when object seen "times" times
  
    Parameters: 
        obj (str): The object to look for
        times (int): The number of times to see the object before continuing
    """
    global lastActions
    client = actionlib.SimpleActionClient(
        "wait", riptide_controllers.msg.WaitAction)
    client.wait_for_server()

    client.send_goal(riptide_controllers.msg.WaitGoal(obj, times))
    lastActions = [client]
    return client

def gateManeuverAction():
    """ 
    Simultaneously translates 360 roll and yaw while driving in direcion of original yaw
    """
    global lastActions
    client = actionlib.SimpleActionClient(
        "gate_maneuver", riptide_controllers.msg.GateManeuverAction)
    client.wait_for_server()

    client.send_goal(riptide_controllers.msg.GateManeuverGoal())
    lastActions = [client]
    return client

def arcAction(angle, velocity, radius):
    """ 
    Move the robot in an arc motion while remaining facing the center

    Parameters: 
        angle (float): How many degrees to move along the circleglobal lastActions
        velocity (float): Velocity in degrees per second
        radius (float): Radius of the circle in which to drive along
    """
    global lastActions
    client = actionlib.SimpleActionClient(
        "arc", riptide_controllers.msg.ArcAction)
    client.wait_for_server()

    client.send_goal(riptide_controllers.msg.ArcGoal(angle, velocity, radius))
    lastActions = [client]
    return client

def alignAction(obj, bboxWidth, hold = False):
    """ 
    Align to the specific object, and keep a rough distance with that object by bboxWidth

    Parameters: 
        obj (str): Name of the object
        bboxWidth (float): Ratio of bbox width over camera width
        hold (bool): Whether to keep aligning after action completes
    """
    global lastActions
    client = actionlib.SimpleActionClient(
        "align", riptide_controllers.msg.AlignAction)
    client.wait_for_server()

    client.send_goal(riptide_controllers.msg.AlignGoal(obj, bboxWidth, hold))
    lastActions = [client]
    return client

def getDistanceAction(obj):
    """ 
    Finds the distance to the specified object in the x axis. Returns distance in meters

    Parameters: 
        obj (str): Name of the object
    """
    global lastActions
    client = actionlib.SimpleActionClient(
        "get_distance", riptide_controllers.msg.GetDistanceAction)
    client.wait_for_server()

    client.send_goal(riptide_controllers.msg.GetDistanceGoal(obj))
    lastActions = [client]
    return client

def searchAction(obj, heading):
    """ 
    Travels in the direction of heading searching for obj

    Parameters: 
        obj (str): Name of the object
        heading (float): direction to travel
    """
    global lastActions
    client = actionlib.SimpleActionClient(
        "search", riptide_autonomy.msg.SearchAction)
    client.wait_for_server()

    client.send_goal(riptide_autonomy.msg.SearchGoal(obj, heading))
    lastActions = [client]
    return client

def gateTaskAction(isLeft):
    """
    The gate task
    
    Parameters: 
        isLeft (bool): Whether the small side of the gate is on the left
    """
    global lastActions
    client = actionlib.SimpleActionClient(
        "gate_task", riptide_autonomy.msg.GateTaskAction)
    client.wait_for_server()

    client.send_goal(riptide_autonomy.msg.GateTaskGoal(isLeft))
    lastActions = [client]
    return client

def buoyTaskAction(isCutieLeft, back):
    """
    The buoy task
    
    Parameters: 
        back (str): Name of the face on the back of the buoys
    """
    global lastActions
    client = actionlib.SimpleActionClient(
        "buoy_task", riptide_autonomy.msg.BuoyTaskAction)
    client.wait_for_server()

    client.send_goal(riptide_autonomy.msg.BuoyTaskGoal(isCutieLeft, back))
    lastActions = [client]
    return client

def decapTaskAction():
    """
    The decap task
    """
    global lastActions
    client = actionlib.SimpleActionClient(
        "decap_task", riptide_autonomy.msg.DecapTaskAction)
    client.wait_for_server()

    client.send_goal(riptide_autonomy.msg.DecapTaskGoal())
    lastActions = [client]
    return client

def garlicTaskAction():
    """
    The Garlic task
    """
    global lastActions
    client = actionlib.SimpleActionClient(
        "garlic_task", riptide_autonomy.msg.GarlicTaskAction)
    client.wait_for_server()

    client.send_goal(riptide_autonomy.msg.GarlicTaskGoal())
    lastActions = [client]
    return client

def exposeTaskAction():
    """
    The Expose task
    """
    global lastActions
    client = actionlib.SimpleActionClient(
        "expose_task", riptide_autonomy.msg.ExposeTaskAction)
    client.wait_for_server()

    client.send_goal(riptide_autonomy.msg.ExposeTaskGoal())
    lastActions = [client]
    return client
