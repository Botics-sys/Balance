from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import math

from controller import *

supervisor = Supervisor()
timestep = int(4)

RArmChain = Chain(name='RArm', links=[
                OriginLink(),
                URDFLink(
                  name="RArmUsy",
                  bounds=[-1.9635,1.9635],
                  translation_vector=[0.024, -0.221, 0.289],
                  orientation=[0,0,0],
                  rotation=[0, 0.5, -0.866025],
                ),
                URDFLink(
                  name="RArmShx",
                  bounds=[-1.74533,1.39626],
                  translation_vector=[0, -0.075, 0.036],
                  orientation=[0,0,0],
                  rotation=[1, 0, 0],
                ),
                URDFLink(
                    name="RArmEly",
                    bounds=[0,3.14159],
                    translation_vector=[0, -0.185,0],
                    orientation=[0,0,0],
                    rotation=[0, 1, 0]
                ),
                URDFLink(
                    name="RArmElx",
                    bounds=[-2.35619, 0],
                    translation_vector=[0, -0.121, 0.013],
                    orientation=[0,1.571,0],
                    rotation=[1, 0, 0]
                ),
                URDFLink(
                    name="RArmUwy",
                    bounds=[-1.571,1.571],
                    translation_vector=[0, -0.188, -0.013],
                    orientation=[0,0,0],
                    rotation=[0, 1, 0]
                ),
                URDFLink(
                    name="RArmMwx",
                    bounds=[-1.571, 0.436],
                    translation_vector=[0, -0.058, 0],
                    orientation=[0,0,0],
                    rotation=[1, 0, 0]
                ) ])
LArmChain = Chain(name='LArm', links=[
                OriginLink(),
                URDFLink(
                  name="LArmUsy",
                  bounds=[-1.9635,1.9635],
                  translation_vector=[0.024, 0.221, 0.289],
                  orientation=[0, 0, 0],
                  rotation=[0, 0.5, 0.866025],
                ),
                URDFLink(
                  name="LArmShx",
                  bounds=[-1.39626,1.74533],
                  translation_vector=[0, 0.075, 0.036],
                  orientation=[0, 0, 0],
                  rotation=[1, 0, 0],
                ),
                URDFLink(
                    name="LArmEly",
                    bounds=[0,3.14159],
                    translation_vector=[0, 0.185, 0],
                    orientation=[0, 0, 0],
                    rotation=[0, 1, 0]
                ),
                URDFLink(
                    name="LArmElx",
                    bounds=[0,2.35619],
                    translation_vector=[0, 0.121, 0.013],
                    orientation=[0, 1.571, 0],
                    rotation=[1, 0, 0]
                ),
                URDFLink(
                    name="LArmUwy",
                    bounds=[-1.571,1.571],
                    translation_vector=[0, 0.188, -0.013],
                    orientation=[0, 0, 0],
                    rotation=[0, 1, 0]
                ),
                URDFLink(
                    name="LArmMwx",
                    bounds=[-0.436, 1.571],
                    translation_vector=[0, 0.058, 0],
                    orientation=[0, 0, 0],
                    rotation=[1, 0, 0]
         )])
RLegChain = Chain(name='RLeg', links=[
                OriginLink(),
                URDFLink(
                  name="RLegUhz",
                  bounds=[-1.14, 0.32],
                  translation_vector=[0, -0.089, 0],
                  orientation=[0, 0, 0],
                  rotation=[0, 0, 1],
                ),
                URDFLink(
                  name="RLegMhx",
                  bounds=[-0.495, 0.47],
                  translation_vector=[0, 0, 0],
                  orientation=[0, 0, 0],
                  rotation=[1, 0, 0],
                ),
                URDFLink(
                    name="RLegLhy",
                    bounds=[-1.745,0.524],
                    translation_vector=[0.05, 0, -0.05],
                    orientation=[0, 0, 0],
                    rotation=[0, 1, 0]
                ),
                URDFLink(
                    name="RLegKny",
                    bounds=[0,2.45],
                    translation_vector=[0.05, 0, -0.374],
                    orientation=[0, 0, 0],
                    rotation=[0, 1, 0]
                ),
                URDFLink(
                    name="RLegUay",
                    bounds=[-0.698, 0.698],
                    translation_vector=[0, 0, -0.422],
                    orientation=[0, 0, 0],
                    rotation=[0, 1, 0]
         )])
LLegChain = Chain(name='LLeg', links=[
                OriginLink(),
                URDFLink(
                  name="LLegUhz",
                  bounds=[-0.32,1.14],
                  translation_vector=[0, 0.089, 0],
                  orientation=[0, 0, 0],
                  rotation=[0, 0, 1],
                ),
                URDFLink(
                  name="LLegMhx",
                  bounds=[-0.47, 0.495],
                  translation_vector=[0, 0, 0],
                  orientation=[0, 0, 0],
                  rotation=[1, 0, 0],
                ),
                URDFLink(
                    name="LLegLhy",
                    bounds=[-1.75,0.524],
                    translation_vector=[0.05, 0, -0.05],
                    orientation=[0, 0, 0],
                    rotation=[0, 1, 0]
                ),
                URDFLink(
                    name="LLegKny",
                    bounds=[0,2.45],
                    translation_vector=[0.05, 0, -0.374],
                    orientation=[0, 0, 0],
                    rotation=[0, 1, 0]
                ),
                URDFLink(
                    name="LLegUay",
                    bounds=[-0.698, 0.698],
                    translation_vector=[0, 0, -0.422],
                    orientation=[0, 0, 0],
                    rotation=[0, 1, 0]
         )])

imuPelvis = supervisor.getInertialUnit('inertial unit 1')
imuPelvis.enable(timestep)

imuTorso = supervisor.getInertialUnit('inertial unit')
imuTorso.enable(timestep)

imuRF = supervisor.getInertialUnit('inertial unit RF')
imuRF.enable(timestep)

imuLF = supervisor.getInertialUnit('inertial unit LF')
imuLF.enable(timestep)

motorsRA = []
motorsLA = []
motorsLL = []
motorsRL = []
motorsB = []

for i in ['RArmUsy', 'RArmShx', 'RArmEly', 'RArmElx','RArmUwy','RArmMwx']:

    motor = supervisor.getMotor(i)
    motor.setVelocity(.6)
    motorsRA.append(motor)
for i in ['LArmUsy', 'LArmShx', 'LArmEly', 'LArmElx','LArmUwy','LArmMwx']:

    motor = supervisor.getMotor(i)
    motor.setVelocity(.6)
    motorsLA.append(motor)
for i in ['LLegUhz', 'LLegMhx', 'LLegLhy', 'LLegKny','LLegUay','LLegLax']:

    motor = supervisor.getMotor(i)
    motor.setVelocity(0.5)
    motorsLL.append(motor)
for i in ['RLegUhz', 'RLegMhx', 'RLegLhy', 'RLegKny','RLegUay','RLegLax']:

    motor = supervisor.getMotor(i)
    motor.setVelocity(0.5)
    motorsRL.append(motor)
for i in ['BackLbz', 'BackMby', 'BackUbx']:

    motor = supervisor.getMotor(i)
    motor.setVelocity(2)
    motorsB.append(motor)

interval = 1

def left_arm(x,y,z):

    LAPos = LArmChain.inverse_kinematics([
        [1,0,0,x],
        [0,1,0,y],
        [0,0,1,z],
        [0,0,0,1]])

    for i in range(5):
        motorsLA[i].setPosition(LAPos[i+1])
def right_arm(x,y,z):

    RAPos = RArmChain.inverse_kinematics([
        [1,0,0,x],
        [0,1,0,y],
        [0,0,1,z],
        [0,0,0,1]])

    for i in range(5):
        motorsRA[i].setPosition(RAPos[i+1])
def left_leg(x,y,z):

    LLPos = LLegChain.inverse_kinematics([
                [1,0,0,x],
                [0,1,0,y],
                [0,0,1,z],
                [0,0,0,1]])

    for i in range(4):
        motorsLL[i].setPosition(LLPos[i+1])
def right_leg(x,y,z):

    RLPos = RLegChain.inverse_kinematics([
                [1,0,0,x],
                [0,1,0,y],
                [0,0,1,z],
                [0,0,0,1]])

    for i in range(4):
        motorsRL[i].setPosition(RLPos[i+1])

## Movement Functions
def balance():

    imuPelvisVals = imuPelvis.getRollPitchYaw()
    imuPelvisPitch = round(imuPelvisVals[1], 3)
    imuPelvisRoll = round(imuPelvisVals[0], 3)


    imuTorsoVals = imuTorso.getRollPitchYaw()
    imuTorsoPitch = round(imuTorsoVals[1], 3)
    imuTorsoRoll = round(imuTorsoVals[0], 3)

    imuRFVals = imuRF.getRollPitchYaw()
    imuRFPitch = round(imuRFVals[1], 3)
    imuRFRoll = round(imuRFVals[0], 3)

    imuLFVals = imuLF.getRollPitchYaw()
    imuLFPitch = round(imuRFVals[1], 3)
    imuLFRoll = round(imuRFVals[0], 3)

    motorsB[1].setPosition(+imuPelvisPitch)

    motorsB[2].setPosition(-imuTorsoRoll)

    motorsRL[4].setPosition(+imuRFPitch - imuPelvisPitch)
    motorsRL[5].setPosition(-imuRFRoll)

    motorsLL[4].setPosition(+imuLFPitch - imuPelvisPitch)
    motorsLL[5].setPosition(-imuLFRoll)


def arms_forward():
    left_arm(0.55, 0.2, 0.1)
    right_arm(0.55,-0.2, 0.1)
def arms_lower():
    left_arm(0.1, 0.25, -0.2)
    right_arm(0.1,-0.25, -0.2)
def arms_right():
    left_arm(0.1, -0.55, -0.2)
    right_arm(0.1, -0.95, 0.2)
def left_arm_left():
    left_arm(0.1, 0.95, 0.2)

def legs_lower1():
    left_leg(0, 0.09, -0.82)
    right_leg(0, -0.09, -0.82)
def legs_lower2():
    left_leg(0, 0.09, -0.8)
    right_leg(0, -0.09, -0.8)
def legs_lower3():
    left_leg(0, 0.09, -0.78)
    right_leg(0, -0.09, -0.78)
def left_leg_lift():
    left_leg(0, 0.09, -0.75)

def lower_legs():

    startTime = supervisor.getTime()
    while supervisor.step(timestep) != -1:
        time = supervisor.getTime()

        balance_pitch()
        if (time - startTime) > 2:
            break

    arms_forward()

    legs_lower1()

    startTime = supervisor.getTime()
    while supervisor.step(timestep) != -1:
        time = supervisor.getTime()

        balance_pitch()
        if (time - startTime) > 1:
            break

    legs_lower2()

    startTime = supervisor.getTime()
    while supervisor.step(timestep) != -1:
        time = supervisor.getTime()

        balance_pitch()
        if (time - startTime) > 1:
            break

    legs_lower3()

    startTime = supervisor.getTime()
    while supervisor.step(timestep) != -1:
        time = supervisor.getTime()

        balance_pitch()
        if (time - startTime) > 3:
            break

def balance_on_left_leg():

    arms_right()

    startTime = supervisor.getTime()
    while supervisor.step(timestep) != -1:
        time = supervisor.getTime()

        balance_pitch()
        if (time - startTime) > 3:
            break

    left_arm_left()

    startTime = supervisor.getTime()
    while supervisor.step(timestep) != -1:
        time = supervisor.getTime()

        balance_pitch()
        if (time - startTime) > 10:
            break

balance_on_left_leg()
