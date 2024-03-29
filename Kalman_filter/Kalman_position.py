import numpy as np 
import matplotlib.pyplot as plt
def draw_Kalman_p(true_track,track,Q,r):

    def kmf_predict(x0,p0):
        a = np.array([[1,0],[0,1]])
        q = np.array([[Q,0],[0,Q]])
        x10 = np.dot(a,x0)
        p10 = np.dot(np.dot(a,p0),a.T)+q

        return x10, p10

    def kmf_update(x10,p10,z):
        H = np.array( [ [ 1, 0 ], [ 0, 1 ] ] )
        R = np.array( [ [ r, 0 ], [0, r] ] )
        di = np.linalg.inv( H.dot(p10).dot(H.T) + R)
        K = p10.dot(H.T).dot(di)
        x =x10 + K.dot( z - H.dot(x10) )
        P = np.eye(len(p10)) - K.dot( H ).dot(p10)
        return x, P, K

    x0 = np.array([0,0])

    p0= np.array([[1,0],[0,1]])

    xi =x0

    pi = p0

    kalman_Filter_result_x=[]
    kalman_Filter_result_y=[]


    time_ax =[]
    var=[]
    error=[]

    for i in range(len(track)):
    

        z = track[i]

        x_minus,P_minus = kmf_predict(xi,pi)


        xi,pi,k =kmf_update(x_minus,P_minus,z)

        kalman_Filter_result_x.append(xi[0])
        kalman_Filter_result_y.append(xi[1])
        time_ax.append(i)
        var.append(pi[1,1])


        # print("pos",i)
        # print("x",i,"=\n",xi)
        # print("p",i,"=\n",pi)
        # print("="*30)


       






    truex = [true_track[i,0] for i in range(len(true_track))]           

    truey = [true_track[i,1] for i in range(len(true_track))]           

    obsx =  [track[i,0] for i in range(len(track))]

    obsy =  [track[i,1] for i in range(len(track))]

       
    for a in range(len(kalman_Filter_result_x)):
        
        err =(((kalman_Filter_result_x[a]-truex[a])**2)+((kalman_Filter_result_y[a]-truey[a])**2))**0.5
        error.append(err)

    # print("error=",error)

    # return error,time_ax,var

    fig=plt.figure(f"position track(Q={Q},R={r})")
    ax1 = fig.add_subplot(1,1,1)
    ax1.plot(truex,truey,'-',label="true track")
    ax1.plot(obsx,obsy,'--',label="Observed track")
    ax1.plot(kalman_Filter_result_x,kalman_Filter_result_y,'-',label="Kalman filter track")
    ax1.legend(loc="best")
    # plt.savefig("tracks")

    fig2=plt.figure("position variance")
    ax2 = fig2.add_subplot(1,1,1)
    ax2.plot(time_ax,var,'-',label="variance")
    ax2.legend(loc="best")
    # plt.savefig("var")
    fig3=plt.figure("position error")

    ax3 = fig3.add_subplot(1,1,1)
    ax3.plot(time_ax,error,'-',label="error")
    ax3.legend(loc="best")
    # plt.savefig("error")
    
    plt.show()
    plt.close()
    

    return error,time_ax,var


