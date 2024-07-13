#add parent dir to find package. Only needed for source code build, pip install doesn't need it.
import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)
import pybullet as p
import math

#from pybullet_envs.bullet.kukaGymEnv import KukaGymEnv
from sawyerEnv import sawyerEnv
import time

def main():

  # replace the values of following 3 variables to recall the last configurations
  # the values are the 3 output of the last run of this program
  handInitial =  [0.26121505230261155, 0.17934456879275282, 1.1146831961129855, 0.26493107969000046, 0.17907812933399364, 0.3545015109713303, 0.22064881672076855, 0.5670891755018503, 1.2468723794210015, 0.18128529903142357, 0.5851992296711084, 0.17014392502411396, 0.5872846661538109, 0.7724645929203886, 0.22290265600560122, 0.6284191039153812, 0.6579502400620709, 0.17077031419079114, 1.1386768699572838, 1.0894212302872914, 0.19751315819872878, 1.1428490439060197, 1.0377958048053253, 0.1714625969253612, 1.5689115835924374, 0.34084727936496095, 0.3406543690380435]
  orientation = [1.571832769393921, 2.837588395277343, 0.8169429206848145]
  palmPosition = [1.1, -0.135, -0.02]
  
  
  #palmPosition = [0.95, 0,0.2]
#handReading =  [1.2155705748026626, 1.3646587415812506, 1.1447266429410867, 1.2164636441775947, 1.3733139839172064, 0.3470031075828074, 1.3302890543350743, 0.7804398654540432, 0.1641915076403882, 1.2478128329559668, 0.9772537851903716, 0.16799841993490994, 1.2000151012117541, 1.5716629299980518, 0.20448325870137504, 1.187444213296405, 1.5608813066138236, 0.17016439712646897, 1.4159326235916034, 1.2264629721139302, 0.17028496185265493, 1.4163177510771723, 1.2293905270141168, 0.1701923382029984, 1.5634067664964022, 0.3398166661011536, 0.34003580355790775]

#orientation =  [1.2892775535583496, 2.827588395276342, 1.2237756252288818]

#palmPosition =  [0.9329789267969317, 0.08350524620618671, 0.25]
  # orientation =  [1.2892775535583496, 2.827588395276342, 1.2237756252288818]
  # palmPosition =  [1, -0.0760530935949646, -0.12]
  # orientation =  [1.2892775535583496, 2.827588395276342, 1.2237756252288818]
  # palmPosition =  [1, -0.0760530935949646, -0.12]
  #Testing #
  # handInitial =  [0.25912000264909263, 0.966522946605846, 0.16993619208974917, 0.2534771907523625, 0.9669597701970489, 0.1699298753615126, 0.582166845271848, 0.6136347448145523, 0.17893924113796303, 0.5824310924395407, 0.5880279689410884, 0.16999744696085142, 0.8648083961998743, 0.65694755007791, 0.2685828795104614, 0.9387343000487742, 0.25464321837406323, 0.1715452626378923, 0.9195652166499492, 0.2071105518470283, 0.17346952669245846, 0.9290640911580923, 0.23144986287077896, 0.1712909036889112, 1.587369655265014, 0.34179098637598926, 0.3444838613635647]

  # orientation =  [1.2892775535583496, 2.827588395276342, 1.2237756252288818]
  # palmPosition =  [0.95, -0.07305263506714255, -0.12]
  environment = sawyerEnv(renders=True, isDiscrete=False, maxSteps=10000000, palmPosition = palmPosition, orientation = orientation)
  readings = [0] * 35
  motorsIds = []
 
  # dv = 0.01
  dv = 0.001
  motorsIds.append(environment._p.addUserDebugParameter("posX", -dv, dv, 0))
  motorsIds.append(environment._p.addUserDebugParameter("posY", -dv, dv, 0))
  dv_z_min = -0.01
  dv_z_max = 0.01
  motorsIds.append(environment._p.addUserDebugParameter("posZ", dv_z_min, dv_z_max, 0))

  # orientation of the palm 
  motorsIds.append(environment._p.addUserDebugParameter("orienX", -math.pi, math.pi, 0))
  motorsIds.append(environment._p.addUserDebugParameter("orienY", -math.pi, math.pi, 0))
  motorsIds.append(environment._p.addUserDebugParameter("orienZ", -math.pi, math.pi, 0))

  #low [0.17 - 1.57], mid [0.34, 1.5]
  motorsIds.append(environment._p.addUserDebugParameter("thumbLow", 0.85, 1.57, handInitial[24]))
  motorsIds.append(environment._p.addUserDebugParameter("thumbMid", 0.34, 1.5, handInitial[25]))
  #[0.17 - 1.57]
  motorsIds.append(environment._p.addUserDebugParameter("indexLow", 0.17, 1.57, handInitial[18]))
  motorsIds.append(environment._p.addUserDebugParameter("indexMid", 0.17, 1.57, handInitial[19]))
  motorsIds.append(environment._p.addUserDebugParameter("middleLow", 0.17, 1.57, handInitial[12]))
  motorsIds.append(environment._p.addUserDebugParameter("middleMid", 0.17, 1.57, handInitial[13]))
  motorsIds.append(environment._p.addUserDebugParameter("ringLow", 0.17, 1.57, handInitial[6]))
  motorsIds.append(environment._p.addUserDebugParameter("ringMid", 0.17, 1.57, handInitial[7]))
  motorsIds.append(environment._p.addUserDebugParameter("pinkyLow", 0.17, 1.57, handInitial[0]))
  motorsIds.append(environment._p.addUserDebugParameter("pinkyMid", 0.17, 1.57, handInitial[1]))
  done = False
  action = []
  while (not done):

    action = []
    for motorId in motorsIds:
      action.append(environment._p.readUserDebugParameter(motorId))

    # print (action)
    # break
    #state, reward, done, info = environment.step2(action)
    state, reward, info = environment.step2(action)
    #environment.step2(action)
    #done = True
    #obs = environment.getExtendedObservation()
    handReading = environment.handReading()
    orientation1 = environment.o()
    palmPosition1 = environment.p()
    palmContact, thumbContact, indexContact, midContact, ringContact, pinkyContact = environment.info()
    qKey = ord('q')
    keys = p.getKeyboardEvents()
    if qKey in keys and keys[qKey]&p.KEY_WAS_TRIGGERED:

      break;

  print("==============================================================================") 
  print("handReading = ", handReading) 
  print("==============================================================================") 
  print("orientation = ", orientation1) 
  print("==============================================================================") 
  print("palmPosition = ", palmPosition1) 
  print("==============================================================================") 
  print("==============================================================================")
  print("==============================================================================") 
  print("==============================================================================")  
  print("PalmCOntact:")
  for x in palmContact:
    print(x)
  print("==============================================================================") 
  print("thumbContact:")
  for x in thumbContact:
    print(x)
  print("==============================================================================") 
  print("indexContact:")
  for x in indexContact:
    print(x)
  print("==============================================================================") 
  print("midContact:")
  for x in midContact:
    print(x)
  print("==============================================================================") 
  print("ringContact:")
  for x in ringContact:
    print(x)
  print("==============================================================================") 
  print("pinkyContact:")
  for x in pinkyContact:
    print(x)

	


if __name__ == "__main__":
  main()


