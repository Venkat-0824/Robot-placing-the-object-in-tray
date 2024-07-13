import pybullet as p
import time
import math
import random
import pybullet_data
from datetime import datetime
import numpy as np

######################################################### Simulation Setup ############################################################################

clid = p.connect(p.GUI)
if (clid < 0):
    p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

planeId = p.loadURDF("plane.urdf", [0, 0, -1])

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
sawyerId = p.loadURDF("./sawyer_robot/sawyer_description/urdf/sawyer.urdf", [0, 0, 0], [0, 0, 0, 3],
                      useFixedBase=1)  # load sawyer robot


tableId = p.loadURDF("./table/table.urdf", [1.1, 0.000000, -0.3],
                     p.getQuaternionFromEuler([(math.pi / 2), 0, (math.pi / 2)]), useFixedBase=1, flags=8)


######################################################### Load Object Here!!!!#############################################################################

# load object, change file name to load different objects
# p.loadURDF(finlename, position([X,Y,Z]), orientation([a,b,c,d]))
# Example:
#objectId = p.loadURDF("random_urdfs/016/016.urdf", [1.25 ,0.25,-0.1], p.getQuaternionFromEuler([0,0,1.56])) # pi*0.5

xpos = 1.1
ypos = 0
ang = 3.14 * 0.5
orn = p.getQuaternionFromEuler([0, 0, ang])

object_path ="random_urdfs/163/163.urdf"
objectId = p.loadURDF(object_path, xpos, ypos, -0.03, orn[0], orn[1], orn[2], orn[3])


######################################################### Load tray Here!!!!#############################################################################

tray_x = 1.0183010196954019
tray_y = 0.3



trayId = p.loadURDF("./tray/tray.urdf", [tray_x, tray_y, 0], [0, 0, 0, 3])



###########################################################################################################################################################


p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.resetBasePositionAndOrientation(sawyerId, [0, 0, 0], [0, 0, 0, 1])

sawyerEndEffectorIndex = 16
numJoints = p.getNumJoints(sawyerId)  # 65 with ar10 hand

# useRealTimeSimulation = 0
# p.setRealTimeSimulation(useRealTimeSimulation)
# p.stepSimulation()
# all R joints in robot
js = [3, 4, 8, 9, 10, 11, 13, 16, 21, 22, 23, 26, 27, 28, 30, 31, 32, 35, 36, 37, 39, 40, 41, 44, 45, 46, 48, 49, 50,
      53, 54, 55, 58, 61, 64]
# lower limits for null space
ll = [-3.0503, -5.1477, -3.8183, -3.0514, -3.0514, -2.9842, -2.9842, -4.7104, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17,
      0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.85, 0.34,
      0.17]
# upper limits for null space
ul = [3.0503, 0.9559, 2.2824, 3.0514, 3.0514, 2.9842, 2.9842, 4.7104, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57,
      0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 2.15, 1.5, 1.5]
# joint ranges for null space
jr = [0, 0, 0, 0, 0, 0, 0, 0, 1.4, 1.4, 1.4, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4,
      0, 1.4, 1.4, 0, 1.3, 1.16, 1.33]
# restposes for null space
rp = [0] * 35
# joint damping coefficents
jd = [1.1] * 35


######################################################### Inverse Kinematics Function ##########################################################################

# Finger tip ID: index:51, mid:42, ring: 33, pinky:24, thumb 62
# Palm ID: 20
# move palm (center point) to reach the target postion and orientation
# input: targetP --> target postion
#        orientation --> target orientation of the palm
# output: joint positons of all joints in the robot
#         control joint to correspond joint position
def palmP(targetP, orientation):
    jointP = [0] * 65
    jointPoses = p.calculateInverseKinematics(sawyerId, 19, targetP, targetOrientation=orientation, jointDamping=jd)
    j = 0
    for i in js:
        jointP[i] = jointPoses[j]
        j = j + 1

    for i in range(p.getNumJoints(sawyerId)):
        p.setJointMotorControl2(bodyIndex=sawyerId,
                                jointIndex=i,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=jointP[i],
                                targetVelocity=0,
                                force=50000,
                                positionGain=0.03,
                                velocityGain=1)
    return jointP


######################################################### Hand Direct Control Functions ##########################################################################

# control the lower joint and middle joint of pinky finger, range both [0.17 - 1.57]

#hand = [21, 22, 23, 26, 27, 28, 30, 31, 32, 35, 36 ,37, 39, 40, 41, 44, 45, 46, 48, 49, 50, 53, 54, 55, 58, 61, 64]
# handReading = [0.2196998776260993, 0.9841056922424084, 0.16991782178342238, 0.21967883521345558, 0.9846229478397389, 0.1699958046620013, 0.5711534611694058, 0.5914229523765463, 0.16999954970542672, 0.573730600144428, 0.5902151809391006, 0.17000660753266578, 0.9359158730554522, 0.265116872922352, 0.170003190706592, 0.9361250259528252, 
# 0.2652466938834658, 0.17003347470289248, 0.9068051254489781, 0.2490975329073341, 0.17008149880963058, 0.9066050389575453, 0.2502858674912193, 0.16999999999999976, 
# 1.5698468053021237, 0.34006621802344955, 0.3400508342876441]

def pinkyF(lower, middle):
    p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[21, 26, 22, 27],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, lower, middle, middle],
                                targetVelocities=[0, 0, 0, 0],
                                forces=[500, 500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1, 1])


# control the lower joint and middle joint of ring finger, range both [0.17 - 1.57]
def ringF(lower, middle):
    p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[30, 35, 31, 36],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, lower, middle, middle],
                                targetVelocities=[0, 0, 0, 0],
                                forces=[500, 500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1, 1])


# control the lower joint and middle joint of mid finger, range both [0.17 - 1.57]
def midF(lower, middle):
    p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[39, 44, 40, 45],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, lower, middle, middle],
                                targetVelocities=[0, 0, 0, 0],
                                forces=[500, 500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1, 1])


# control the lower joint and middle joint of index finger, range both [0.17 - 1.57]
def indexF(lower, middle):
    p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[48, 53, 49, 54],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, lower, middle, middle],
                                targetVelocities=[0, 0, 0, 0],
                                forces=[500, 500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1, 1])


# control the lower joint and middle joint of thumb, range: low [0.17 - 1.57], mid [0.34, 1.5]
def thumb(lower, middle):
    p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[58, 61, 64],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, middle, middle],
                                targetVelocities=[0, 0, 0],
                                forces=[500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1])




  

##################################################################### Input Value Here############################################################

handInitial =  [0.26121505230261155, 0.17934456879275282, 1.1146831961129855, 0.26493107969000046, 0.17907812933399364, 0.3545015109713303, 0.22064881672076855, 0.5670891755018503, 1.2468723794210015, 0.18128529903142357, 0.5851992296711084, 0.17014392502411396, 0.5872846661538109, 0.7724645929203886, 0.22290265600560122, 0.6284191039153812, 0.6579502400620709, 0.17077031419079114, 1.1386768699572838, 1.0894212302872914, 0.19751315819872878, 1.1428490439060197, 1.0377958048053253, 0.1714625969253612, 1.5689115835924374, 0.34084727936496095, 0.3406543690380435]
grasp_orientation = [1.571832769393921, 2.837588395277343, 0.8169429206848145]
grasp_palmPosition = [1.1, -0.135, -0.02]
handClose =   [0.9518602488025184, 0.9525470182946459, 1.5474074983539889, 0.9360509968123255, 0.9350156258164676, 0.22392050064941072, 0.5944632666102456, 0.5793631323758753, 1.16989640098739248, 0.5854447548559536, 0.5821846276001464, 0.17047160298302724, 0.8805923372517218, 0.3838697556762429, 0.22621806265375474, 0.9384265748909743, 0.2984993223313543, 0.17173215163837467, 1.2230835819966675, 0.22276218388972487, 0.17234180343293869, 1.2271037262924913, 0.24140473736653117, 0.17141295627521993, 1.5777646827634443, 0.5456018336264689, 0.5480414005648172]
pu_palmPosition =  [0.94, 0.0, 0.25]
pu_orientation  =  [1.2892775535683597, 2.827588395376452, 1.2237756253388918]
final_palmPosition = [0.9110235933478783, 0.17, 0.25]
final_orientation = [0.727097749711193, 2.2984782021237885, 0.4631795893178812]
handOpen = [0.23791776350578354, 0.1757544910936571, 0.29764745486837116, 0.23754362279773994, 0.16999597089686969, 0.3775939322409635, 0.18168474927512803, 0.1758926369336866, 0.1845598545607197, 0.17030196791446582, 0.17011452285795407, 0.1700102825769791, 0.1717451657212141, 0.17208485272719378, 0.17001139107791998, 0.17009669284461805, 0.17393682045571487, 0.16999999988999997, 0.170107095758437, 0.26215783072686536, 0.16999960893597742, 0.17000115527206383, 0.25230788515373766, 0.1700390287977374, 0.8499999370367525, 0.3401734466845466, 0.3400368734419835]
##################################################################################################################################################################################################
initial_palmPosition = [0.94, 0.0, 0.2]
initial_orientation =  [1.2892775535583496, 2.827588395276342, 1.2237756252288818]

initial_palmPosition = [initial_palmPosition[0]-0.1,initial_palmPosition[1]-0.05,initial_palmPosition[2]]
##################################################################################################################################################################################################
p.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=0, cameraPitch=-150, cameraTargetPosition=[0.8,0.5,0.2])
p.setGravity(0, 0, -10)

# write the code for step 9-12


''' Step 1: move robot to the initial position '''

# Parameters: 
	# arm: initial_palmPosition, initial_orientation
	# hand: handInitial


''' Step 2: move robot to the grasping position '''

# Parameters: 
	# arm: grasp_orientation, grasp_palmPosition


''' Step 3: grasp object '''

# Parameters: 
	# hand: handClose

''' Step 4: pick up  the object '''

# Parameters: 
	# arm: pu_palmPosition, pu_orientation

''' Step 4: move object to the tray '''

# Parameters: 
	# arm: final_palmPosition, final_orientation


''' Step 5: put the object in the tray '''

# Parameters: 
	# hand: handOpen

#inital position
k = 0
while 1:
    k = k + 1

    # move palm to target postion
    i = 0
    while 1:
        i += 1
        # p.stepSimulation()
        currentP = palmP([initial_palmPosition[0], initial_palmPosition[1], initial_palmPosition[2]],
                         p.getQuaternionFromEuler([initial_orientation[0], initial_orientation[1], initial_orientation[2]]))
        time.sleep(0.03)
        p.stepSimulation()

        if (i == 100):
            print('robot reached initial position')
            break  
        # moving palm to grasp position.
 



    # moving palm to target postion
    i = 0
    while 1:
        i += 1
        #p.stepSimulation()
        currentP = palmP([1.0, -0.005, -0.12],
                         p.getQuaternionFromEuler([1.4292775535583496, grasp_orientation[1], grasp_orientation[2]]))
        #low [0.17 - 1.57], mid [0.34, 1.5]
        #thumb(-1.57,0.34)
        
       
                
        time.sleep(0.03)
        p.stepSimulation()

        if (i == 100):
            print(' Robot grasped object position')
            break  
        #handclose
    i = 0
    while 1:
        i+=1
        thumb(1.5, 0.5)
        indexF(1.5, 1.57)
        midF(0.5, 1.57)
        ringF(0.5,1.57)
        pinkyF(0.9, 1.57)
        time.sleep(0.03)
        p.stepSimulation()

        if (i == 100):
            print('Object position closed suceefully')
            break 
        # pickup
    i = 0
    while 1:
        i += 1
        # p.stepSimulation()
        currentP = palmP([pu_palmPosition[0], pu_palmPosition[1], pu_palmPosition[2]],
                         p.getQuaternionFromEuler([pu_orientation[0], pu_orientation[1], pu_orientation[2]]))
        time.sleep(0.03)
        p.stepSimulation()

        if (i == 100):
            print('picking up object')
            break  
        # object to tray
    i = 0
    while 1:
        i += 1
        p.stepSimulation()
        currentP = palmP([0.7110235933378673, 0.30, -0.25],
                         p.getQuaternionFromEuler([1.0546246528625488, 2.827698495276342, 1.2238756252288818]))
        #thumb(-1.57, 1.5)
        #indexF(handOpen[2], handOpen[3])
        thumb(-1.57, -1.57)
        indexF(-1.57, -1.57)
        midF(-1.57, -1.57)
        ringF(-1.57, -1.57)
        pinkyF(-1.57, -1.57)
        time.sleep(0.03)

        time.sleep(0.03)
        p.stepSimulation()

        if (i == 100):
            print('move object to tray')
            break  
        #handopen
    i = 0
    '''while 1:
        i += 1
        thumb(handOpen[0], handOpen[1])
        indexF(handOpen[2], handOpen[3])
        midF(handOpen[4], handOpen[5])
        ringF(handOpen[6], handOpen[7])
        pinkyF(handOpen[8], handOpen[9])
        time.sleep(0.03)
        p.stepSimulation()
        

        if (i == 100):
            print('object left in tray successfully')
            break 
        '''







p.disconnect()
print("disconnected")