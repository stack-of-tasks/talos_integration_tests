from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.core.meta_tasks_kine import (MetaTaskKine6d, MetaTaskKineCom)
from numpy import eye


def init_appli(robot):
    taskRH = MetaTaskKine6d('rh', robot.dynamic, 'rh', robot.OperationalPointsMap['right-wrist'])
    handMgrip = eye(4)
    handMgrip[0:3, 3] = (0.1, 0, 0)
    taskRH.opmodif = matrixToTuple(handMgrip)
    taskRH.feature.frame('desired')
    # --- STATIC COM (if not walking)
    taskCom = MetaTaskKineCom(robot.dynamic)
    robot.dynamic.com.recompute(0)
    taskCom.featureDes.errorIN.value = robot.dynamic.com.value
    taskCom.task.controlGain.value = 10

    # --- CONTACTS
    contactLF = MetaTaskKine6d('contactLF', robot.dynamic, 'LF', robot.OperationalPointsMap['left-ankle'])
    contactLF.feature.frame('desired')
    contactLF.gain.setConstant(10)
    contactLF.keep()
    locals()['contactLF'] = contactLF

    contactRF = MetaTaskKine6d('contactRF', robot.dynamic, 'RF', robot.OperationalPointsMap['right-ankle'])
    contactRF.feature.frame('desired')
    contactRF.gain.setConstant(10)
    contactRF.keep()
    locals()['contactRF'] = contactRF

    from dynamic_graph import plug
    from dynamic_graph.sot.core.sot import SOT
    sot = SOT('sot')
    sot.setSize(robot.dynamic.getDimension())
    plug(sot.control, robot.device.control)

    from dynamic_graph.ros import RosPublish
    ros_publish_state = RosPublish("ros_publish_state")
    ros_publish_state.add("vector", "state", "/sot_control/state")
    from dynamic_graph import plug
    plug(robot.device.state, ros_publish_state.state)
    robot.device.after.addDownsampledSignal("ros_publish_state.trigger", 100)

    sot.push(contactRF.task.name)
    sot.push(contactLF.task.name)
    sot.push(taskCom.task.name)
    robot.device.control.recompute(0)
