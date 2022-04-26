import math
import matplotlib.pyplot as plt
import random 
import matplotlib.patches as mpatches




def create_env(h,w,Obstacles):
    obs_h=0.1
    obs_w=0.1
    obs=[]
    while (len(obs) < Obstacles ):
        y = random.random()*(h-obs_h)
        x = random.random()*(w-obs_w)
        if [y,x] not in obs:
            obs.append( [ y, x, obs_h, obs_w] )
    return h, w, obs 



def draw_environment( ax, env ):
  frame = []
  height, width, obs = env
  ax.set_ylim((0,height))
  ax.set_xlim((0,width))
 

  for y, x, sy, sx in obs:
  
    r = mpatches.Rectangle( (x,y), sx, sy,random.gauss(90,45), color = 'brown' )
    a = ax.add_patch( r )
    frame.append( a )



    
  return frame

# h,w,obs= create_env(9,3,40)
# print(obs)
env = create_env(9,3,20)
# print(env[0],env[1])



class robot:


  def __init__(self,x,y,theta):
    self.x = x
    self.y = y
    self.theta=robot.normalizeAngle(theta)    
  def normalizeAngle(angle):
    while angle >= 2.0 * math.pi:
      angle = angle - 2.0 * math.pi
    while angle < 0:
      angle = angle + 2.0 * math.pi
    return angle
  def move(self,dist):
    stddev_m = 0.1      
    dist= random.gauss(dist,stddev_m*dist)
    x = self.x+math.cos(self.theta)*dist
    y = self.y+math.sin(self.theta)*dist
    self.x=x
    self.y=y
  def turn(self,angle):
    stddev_a = 0.01
    angle=self.theta+random.gauss(angle,stddev_a*angle)
    self.theta=robot.normalizeAngle(angle)
  def sensor_model(self,env):
    h,w,obs=env
    y,x=self.y,self.x
    robot_theta = self.theta
    sense = []
    for obsy1, obsx1, obsh, obsw in obs:
        obsy, obsx = obsy1 + 0.5 * obsh, obsx1 + 0.5 * obsw
        dist = math.hypot( obsy-y, obsx - x )
        sense_angle =robot.normalizeAngle(math.atan2(obsy-y,obsx-x))
        if sense_angle <= robot_theta + (40/180)*math.pi and sense_angle >= robot_theta - (40/180)*math.pi:
          if dist >=0 and dist < 0.5 :
            r=random.random()
            if r < 0.6:
              tdist = dist+random.gauss(0,0.05*dist)
              tangle = robot.normalizeAngle(sense_angle + random.gauss( 0,dist*0.1 )-robot_theta+0.5*math.pi )
              re_x,re_y=tdist*math.cos(tangle),tdist*math.sin(tangle)
              sense.append([tdist,tangle,re_x,re_y,obsy,obsx])
          elif dist>=0.5 and dist<1:
            r=random.random()
            if r < 0.8:
              tdist = dist+random.gauss(0,0.01*dist)
              tangle = robot.normalizeAngle(sense_angle + random.gauss( 0,dist*0.1 )-robot_theta+0.5*math.pi )
              re_x,re_y=tdist*math.cos(tangle),tdist*math.sin(tangle)
              sense.append([tdist,tangle,re_x,re_y,obsy,obsx])
          elif dist >=1 :
            r = random.random()
            if r <0.1:
              tdist = dist+random.gauss(0,0.1*dist)
              tangle = robot.normalizeAngle(sense_angle + random.gauss( 0,dist*0.1 )-robot_theta+0.5*math.pi )
              re_x,re_y=tdist*math.cos(tangle),tdist*math.sin(tangle)
              sense.append([tdist,tangle,re_x,re_y,obsy,obsx])  
    fake_dist = random.random()*1.5
    r2 = random.gauss(robot_theta,40/180*math.pi)
    fake_y ,fake_x=y+math.sin(r2)*fake_dist,x+math.cos(r2)*fake_dist
    fake_angle=robot.normalizeAngle(math.atan2(fake_y-y,fake_x-x))
    if fake_angle>=robot_theta-40/180*math.pi and  fake_angle<=robot_theta+40/180*math.pi:
      if fake_dist >=0 and fake_dist < 0.5 :
          r=random.random()
          if r < 0.01:
            re_fakex,re_fakey=fake_dist*math.cos(fake_angle),fake_dist*math.sin(fake_angle)
            sense.append( [ fake_dist, fake_angle,re_fakex,re_fakey,fake_y ,fake_x ] )
      elif fake_dist>=0.5 and fake_dist<1:
          r=random.random()
          if r < 0.1:    
            re_fakex,re_fakey=fake_dist*math.cos(fake_angle),fake_dist*math.sin(fake_angle) 
            sense.append( [fake_dist, fake_angle,re_fakex, re_fakey,fake_y ,fake_x ] )
      elif fake_dist >=1 and fake_dist <=1.5:
          r = random.random()
          if r <0.05:
            re_fakex,re_fakey=fake_dist*math.cos(fake_angle),fake_dist*math.sin(fake_angle)
            sense.append( [ fake_dist, fake_angle,re_fakex, re_fakey,fake_y ,fake_x] )
    return sense
  def gauss(mu,sigma,x):
    g = math.e**(- ((mu - x) ** 2) / (sigma ** 2)/2.0)/math.sqrt(2.0 *math. pi * (sigma ** 2))
    return g
  def calculate_prob(self,measurement):
    prob = 1
    for i in range(len(measurement)):
        dist = math.hypot(self.y-measurement[i][4],self.x-measurement[i][5])
        prob *= robot.gauss(dist,0.5,measurement[i][0])
    return prob
for N in [1000,100,10]:
  my_robot_x,my_robot_y,my_robot_theta = random.random()*env[1],random.random()*env[0],2*math.pi*random.random()
  my_robot=robot(my_robot_x,my_robot_y,my_robot_theta)
  # print(["robot x =",my_robot.x,"robot y=",my_robot.y,"my robot theta = ",my_robot_theta])




  fig = plt.figure(f"{N} particles")
  ax1 = fig.add_subplot(1,3,1)
  ax1.set_title("before resample")
  ax2 = fig.add_subplot(1,3,2)
  ax2.set_title("1 move")
  ax3 = fig.add_subplot(1,3,3)
  ax3.set_title("many moves")
  draw_environment(ax1,env)
  draw_environment(ax2,env)
  draw_environment(ax3,env)


 

  def particle_filter(Nums_of_particle):
    my_robot.turn(0)
    my_robot.move(0.1)
    if my_robot.x>3:
      my_robot.x=my_robot.x-env[1]
    if my_robot.y>9:
      my_robot.y=my_robot.y-env[0]  
    if my_robot.x<0:
      my_robot.x=env[1]+my_robot.x
    if my_robot.y<0:
      my_robot.y=env[0]+my_robot.y
    

    my_robot_sense = my_robot.sensor_model(env)

    count = 1
    while my_robot_sense==[]:
      my_robot.turn(math.pi*0.5)
      my_robot.move(0.1)
      if my_robot.x>3:
        my_robot.x=my_robot.x-env[1]
      if my_robot.y>9:
        my_robot.y=my_robot.y-env[0]  
      if my_robot.x<0:
        my_robot.x=env[1]+my_robot.x
      if my_robot.y<0:
        my_robot.y=env[0]+my_robot.y
      my_robot_sense = my_robot.sensor_model(env)
      count+=1
    for i in range(len(my_robot_sense)):
      print("distance to the center of the wall=",my_robot_sense[i][0],"\nrelative angle to the center of the wall=",str(round(my_robot_sense[i][1]*180/math.pi,2))+"°")
      print("the wall relative orientation to the robot="+str((my_robot_sense[i][2],my_robot_sense[i][3])))
                                                                                                      
    p = []
    for i in range(Nums_of_particle):
      x,y,theta = random.random()*env[1],random.random()*env[0],random.random()*math.pi*2
      particle_pos= robot(x,y,theta)
      p.append([particle_pos.x,particle_pos.y,particle_pos.theta])    
    p_move=[]
    for i in range(Nums_of_particle):
      particle_move = robot(p[i][0],p[i][1],p[i][2])
      # print("移動前",[particle_move.x,particle_move.y,particle_move.theta])
      particle_move.turn(0)
      particle_move.move(0.1*count)
      # print("移動後",[particle_move.x,particle_move.y,particle_move.theta])
      p_move.append([particle_move.x,particle_move.y,particle_move.theta])
    w = []
    for i in range(Nums_of_particle):
      particle_move2 = robot(p_move[i][0],p_move[i][1],p_move[i][2])
      w.append(particle_move2.calculate_prob(my_robot_sense))
    
    
    p3 = []
    w_max=max(w)
    beta = 0
    index=random.randint(1,Nums_of_particle)
    for i in range(Nums_of_particle):
      beta += random.random()*2* w_max
      while beta > w[index]:
        beta -= w[index]
        index = (index + 1)%Nums_of_particle  
      p3.append(p[index])
    return p,p3


  particle,resample_particle=particle_filter(N)

  x1pt = list(particle[i][0] for i in range(len(particle))) 
  y1pt = list(particle[i][1] for i in range(len(particle))) 
  ax1.scatter(x1pt,y1pt,10,c="blue")
  ax1.scatter(my_robot.x,my_robot.y,50,c="red",cmap="plasma")





  x2pt = list(resample_particle[i][0] for i in range(len(particle))) 
  y2pt = list(resample_particle[i][1] for i in range(len(particle))) 
  ax2.scatter(x2pt,y2pt,10,c="blue")
  ax2.scatter(my_robot.x,my_robot.y,50,c="red",cmap="plasma")



  def resample(p):
    m=0.1
    my_robot.turn(0)
    my_robot.move(m)
    if my_robot.x>3:
      my_robot.x=my_robot.x-env[1]
    if my_robot.y>9:
      my_robot.y=my_robot.y-env[0]  
    if my_robot.x<0:
      my_robot.x=env[1]+my_robot.x
    if my_robot.y<0:
      my_robot.y=env[0]+my_robot.y    
    my_robot_sense2 = my_robot.sensor_model(env)
    
    count = 1
    while my_robot_sense2==[]:
      my_robot.turn(0)
      my_robot.move(m)
      if my_robot.x>3:
        my_robot.x=my_robot.x-env[1]
      if my_robot.y>9:
        my_robot.y=my_robot.y-env[0]  
      if my_robot.x<0:
        my_robot.x=env[1]+my_robot.x
      if my_robot.y<0:
        my_robot.y=env[0]+my_robot.y
      my_robot_sense2 = my_robot.sensor_model(env)
      count+=1
    

    for i in range(int(len(p)*0.1)):
      x,y,theta = random.random()*env[1],random.random()*env[0],random.random()*math.pi*2
      particle_pos= robot(x,y,theta)
      p.append([particle_pos.x,particle_pos.y,particle_pos.theta])    
    p_move2=[]
    for i in range(len(p)):
      particle_move2 = robot(p[i][0],p[i][1],p[i][2])
      # print("移動前",[particle_move.x,particle_move.y,particle_move.theta])
      particle_move2.turn(0)
      particle_move2.move(0.1*count)
      # print("移動後",[particle_move.x,particle_move.y,particle_move.theta])
      p_move2.append([particle_move2.x,particle_move2.y,particle_move2.theta])
    w = []
    for i in range(len(p)):
      particle_move3 = robot(p_move2[i][0],p_move2[i][1],p_move2[i][2])
      w.append(particle_move3.calculate_prob(my_robot_sense2))
    
    # print("weight=",w)   
    p3 = []
    w_max=max(w)
    beta = 0
    index=random.randint(0,len(p))
    for i in range(N):
      beta += random.random()*2* w_max
      while beta > w[index]:
        beta -= w[index]
        index = (index + 1)%len(p)
      p3.append(p[index])
    return p3




  if N ==10:

    for i in range(1):
      resample_particle=resample(resample_particle)

  elif N==100:   
    for i in range(5):
      resample_particle=resample(resample_particle)
  else :   
    for i in range(30):
      resample_particle=resample(resample_particle)
  my_robot_sense =my_robot.sensor_model(env)
  for i in range(len(my_robot_sense)):
      print("after resample,distance to the center of the wall=",my_robot_sense[i][0],"\nafter resample,relative angle to the center of the wall=",str(round(my_robot_sense[i][1]*180/math.pi,2))+"°")
      print("after resample,the wall relative orientation to the robot="+str((my_robot_sense[i][2],my_robot_sense[i][3])))


  x3pt = list(resample_particle[i][0] for i in range(len(resample_particle))) 
  y3pt = list(resample_particle[i][1] for i in range(len(resample_particle))) 



  ax3.scatter(x3pt,y3pt,10,c="blue")
  ax3.scatter(my_robot.x,my_robot.y,50,c="red",cmap="plasma")
  plt.show()
  plt.close()

