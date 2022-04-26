import numpy as np
import matplotlib.pyplot as plt
from Kalman_position import draw_Kalman_p
from Kalman_acceleration import draw_Kalman_a
from Kalman_velocity import draw_Kalman_V

true_track = np.array([[ 1.00000000e+00  ,0.00000000e+00],
 [ 1.99950656e+00 , 3.14107591e-02],[ 2.99753329e+00  ,9.42012786e-02], [ 3.99309525e+00 , 1.88309592e-01],[ 4.98520995e+00 , 3.13642825e-01],[ 5.97289830e+00 , 4.70077291e-01],
 [ 6.95518555e+00 , 6.57458605e-01], [ 7.93110231e+00 , 8.75601847e-01],
 [ 8.89968547e+00 , 1.12429173e+00],
 [ 9.85997915e+00  ,1.40328284e+00],
 [ 1.08110357e+01 ,1.71229983e+00],
 [ 1.17519164e+01  ,2.05103775e+00],
 [ 1.26816929e+01  ,2.41916231e+00],
 [ 1.35994476e+01  ,2.81631020e+00],
 [ 1.45042746e+01  ,3.24208949e+00],
 [ 1.53952811e+01  ,3.69607999e+00],
 [ 1.62715878e+01  ,4.17783366e+00],
 [ 1.71323298e+01  ,4.68687508e+00],
 [ 1.79766578e+01  ,5.22270187e+00],
 [ 1.88037383e+01  ,5.78478525e+00],
 [ 1.96127553e+01  ,6.37257050e+00],
 [  2.04029103e+01  ,6.98547756e+00],
 [   2.11734236e+01  ,7.62290155e+00],
 [  2.19235347e+01  ,8.28421341e+00],
 [   2.26525033e+01  ,8.96876052e+00],
 [ 2.33596101e+01  ,9.67586730e+00],
 [ 2.40441572e+01  ,1.04048359e+01],
 [ 2.47054690e+01  ,1.11549470e+01],
 [ 2.53428930e+01  ,1.19254602e+01],
 [ 2.59558001e+01  ,1.27156153e+01],
 [ 2.65435853e+01  ,1.35246322e+01],
 [ 2.71056687e+01  ,1.43517128e+01],
 [ 2.76414955e+01  ,1.51960407e+01],
 [ 2.81505369e+01  ,1.60567828e+01],
 [ 2.86322906e+01  ,1.69330895e+01],
 [ 2.90862811e+01  ,1.78240960e+01],
 [ 2.95120604e+01  ,1.87289230e+01],
 [ 2.99092083e+01  ,1.96466777e+01],
 [ 3.02773328e+01  ,2.05764541e+01],
 [ 3.06160707e+01  ,2.15173349e+01], 
 [ 3.09250877e+01  ,2.24683914e+01],
 [ 3.12040788e+01  ,2.34286851e+01],
 [ 3.14527687e+01  ,2.43972683e+01],
 [ 3.16709120e+01  ,2.53731850e+01],
 [ 3.18582933e+01  ,2.63554723e+01],
 [ 3.20147278e+01  ,2.73431606e+01],
 [ 3.21400610e+01  ,2.83352753e+01],
 [ 3.22341693e+01  ,2.93308373e+01],
 [ 3.22969598e+01  ,3.03288640e+01],
 [ 3.23283706e+01  ,3.13283706e+01],
 [ 3.23283706e+01  ,3.23283706e+01],
 [ 3.22969598e+01  ,3.33278771e+01],
 [ 3.22341693e+01  ,3.43259039e+01],
 [ 3.21400610e+01  ,3.53214658e+01],
 [ 3.20147278e+01  ,3.63135805e+01],
 [ 3.18582933e+01  ,3.73012689e+01],
 [ 3.16709120e+01  ,3.82835561e+01],
 [ 3.14527687e+01  ,3.92594729e+01],
 [ 3.12040788e+01  ,4.02280561e+01],
 [ 3.09250877e+01  ,4.11883497e+01],
 [ 3.06160707e+01  ,4.21394063e+01],
 [ 3.02773328e+01  ,4.30802870e+01],
 [ 2.99092083e+01  ,4.40100635e+01],
 [ 2.95120604e+01  ,4.49278181e+01],
 [ 2.90862811e+01  ,4.58326452e+01],
 [ 2.86322906e+01  ,4.67236517e+01],
 [ 2.81505369e+01  ,4.75999584e+01],
 [ 2.76414955e+01  ,4.84607004e+01],
 [ 2.71056687e+01  ,4.93050283e+01],
 [ 2.65435853e+01  ,5.01321089e+01],
 [ 2.59558001e+01  ,5.09411259e+01],
 [ 2.53428930e+01  ,5.17312809e+01],
 [ 2.47054690e+01  ,5.25017942e+01],
 [ 2.40441572e+01  ,5.32519052e+01],
 [ 2.33596101e+01  ,5.39808739e+01],
 [ 2.26525033e+01  ,5.46879806e+01],
 [ 2.19235347e+01  ,5.53725278e+01],
 [ 2.11734236e+01  ,5.60338396e+01],
 [ 2.04029103e+01  ,5.66712636e+01],
 [ 1.96127553e+01  ,5.72841707e+01],
 [ 1.88037383e+01  ,5.78719559e+01],
 [ 1.79766578e+01  ,5.84340393e+01],
 [ 1.71323298e+01  ,5.89698661e+01],
 [ 1.62715878e+01  ,5.94789075e+01],
 [ 1.53952811e+01  ,5.99606612e+01],
 [ 1.45042746e+01  ,6.04146517e+01],
 [ 1.35994476e+01  ,6.08404310e+01],
 [ 1.26816929e+01  ,6.12375789e+01],
 [ 1.17519164e+01  ,6.16057034e+01],
 [ 1.08110357e+01  ,6.19444413e+01],
 [ 9.85997915e+00  ,6.22534583e+01],
 [ 8.89968547e+00  ,6.25324494e+01],
 [ 7.93110231e+00  ,6.27811393e+01],
 [ 6.95518555e+00  ,6.29992826e+01],
 [ 5.97289830e+00  ,6.31866639e+01],
 [ 4.98520995e+00  ,6.33430983e+01],
 [ 3.99309525e+00  ,6.34684316e+01],
 [ 2.99753329e+00  ,6.35625399e+01],
 [ 1.99950656e+00  ,6.36253304e+01],
 [ 1.00000000e+00  ,6.36567412e+01],
 [-2.13162821e-14  ,6.36567412e+01],
 [-9.99506560e-01  ,6.36253304e+01],
 [-1.99753329e+00  ,6.35625399e+01],
 [-2.99309525e+00  ,6.34684316e+01],
 [-3.98520995e+00  ,6.33430983e+01],
 [-4.97289830e+00  ,6.31866639e+01],
 [-5.95518555e+00  ,6.29992826e+01],
 [-6.93110231e+00  ,6.27811393e+01],
 [-7.89968547e+00  ,6.25324494e+01],
 [-8.85997915e+00  ,6.22534583e+01],
 [-9.81103567e+00  ,6.19444413e+01],
 [-1.07519164e+01  ,6.16057034e+01],
 [-1.16816929e+01  ,6.12375789e+01],
 [-1.25994476e+01  ,6.08404310e+01],
 [-1.35042746e+01  ,6.04146517e+01],
 [-1.43952811e+01  ,5.99606612e+01],
 [-1.52715878e+01  ,5.94789075e+01],
 [-1.61323298e+01  ,5.89698661e+01],
 [-1.69766578e+01  ,5.84340393e+01],
 [-1.78037383e+01  ,5.78719559e+01],
 [-1.86127553e+01  ,5.72841707e+01],
 [-1.94029103e+01  ,5.66712636e+01],
 [-2.01734236e+01  ,5.60338396e+01],
 [-2.09235347e+01  ,5.53725278e+01],
 [-2.16525033e+01  ,5.46879806e+01],
 [-2.23596101e+01  ,5.39808739e+01],
 [-2.30441572e+01  ,5.32519052e+01],
 [-2.37054690e+01  ,5.25017942e+01],
 [-2.43428930e+01  ,5.17312809e+01],
 [-2.49558001e+01  ,5.09411259e+01],
 [-2.55435853e+01  ,5.01321089e+01],
 [-2.61056687e+01  ,4.93050283e+01],
 [-2.66414955e+01  ,4.84607004e+01],
 [-2.71505369e+01  ,4.75999584e+01],
 [-2.76322906e+01  ,4.67236517e+01],
 [-2.80862811e+01  ,4.58326452e+01],
 [-2.85120604e+01  ,4.49278181e+01],
 [-2.89092083e+01  ,4.40100635e+01],
 [-2.92773328e+01  ,4.30802870e+01],
 [-2.96160707e+01  ,4.21394063e+01],
 [-2.99250877e+01  ,4.11883497e+01],
 [-3.02040788e+01  ,4.02280561e+01],
 [-3.04527687e+01  ,3.92594729e+01],
 [-3.06709120e+01  ,3.82835561e+01],
 [-3.08582933e+01  ,3.73012689e+01],
 [-3.10147278e+01  ,3.63135805e+01],
 [-3.11400610e+01  ,3.53214658e+01],
 [-3.12341693e+01  ,3.43259039e+01],
 [-3.12969598e+01  ,3.33278771e+01],
 [-3.13283706e+01  ,3.23283706e+01],
 [-3.13283706e+01  ,3.13283706e+01],
 [-3.12969598e+01  ,3.03288640e+01],
 [-3.12341693e+01  ,2.93308373e+01],
 [-3.11400610e+01  ,2.83352753e+01],
 [-3.10147278e+01  ,2.73431606e+01],
 [-3.08582933e+01  ,2.63554723e+01],
 [-3.06709120e+01  ,2.53731850e+01],
 [-3.04527687e+01  ,2.43972683e+01],
 [-3.02040788e+01  ,2.34286851e+01],
 [-2.99250877e+01  ,2.24683914e+01],
 [-2.96160707e+01  ,2.15173349e+01],
 [-2.92773328e+01  ,2.05764541e+01],
 [-2.89092083e+01  ,1.96466777e+01],
 [-2.85120604e+01  ,1.87289230e+01],
 [-2.80862811e+01  ,1.78240960e+01],
 [-2.76322906e+01  ,1.69330895e+01],
 [-2.71505369e+01  ,1.60567828e+01],
 [-2.66414955e+01  ,1.51960407e+01],
 [-2.61056687e+01  ,1.43517128e+01],
 [-2.55435853e+01  ,1.35246322e+01],
 [-2.49558001e+01  ,1.27156153e+01],
 [-2.43428930e+01  ,1.19254602e+01],
 [-2.37054690e+01  ,1.11549470e+01],
 [-2.30441572e+01  ,1.04048359e+01],
 [-2.23596101e+01  ,9.67586730e+00],
 [-2.16525033e+01  ,8.96876052e+00],
 [-2.09235347e+01  ,8.28421341e+00],
 [-2.01734236e+01  ,7.62290155e+00],
 [-1.94029103e+01  ,6.98547756e+00],
 [-1.86127553e+01  ,6.37257050e+00],
 [-1.78037383e+01  ,5.78478525e+00],
 [-1.69766578e+01  ,5.22270187e+00],
 [-1.61323298e+01  ,4.68687508e+00],
 [-1.52715878e+01  ,4.17783366e+00],
 [-1.43952811e+01  ,3.69607999e+00],
 [-1.35042746e+01  ,3.24208949e+00],
 [-1.25994476e+01  ,2.81631020e+00],
 [-1.16816929e+01  ,2.41916231e+00],
 [-1.07519164e+01  ,2.05103775e+00],
 [-9.81103567e+00  ,1.71229983e+00],
 [-8.85997915e+00  ,1.40328284e+00],
 [-7.89968547e+00  ,1.12429173e+00],
 [-6.93110231e+00  ,8.75601847e-01],
 [-5.95518555e+00  ,6.57458605e-01],
 [-4.97289830e+00  ,4.70077291e-01],
 [-3.98520995e+00  ,3.13642825e-01],
 [-2.99309525e+00  ,1.88309592e-01],
 [-1.99753329e+00  ,9.42012786e-02],
 [-9.99506560e-01  ,3.14107591e-02],
 [-8.85180818e-13 ,-2.97456504e-13]])         

Observed_Track_1=np.array([[ 1.65675138e+00 , 1.04948187e-01],
 [ 1.04340664e+00 ,-8.03613207e-01],
 [ 3.32975888e+00  ,2.70582517e-01],
 [ 3.46540198e+00  ,2.42796077e-01],
 [ 5.09636258e+00  ,7.14719724e-01],
 [ 6.17796887e+00  ,2.40186652e+00],
 [ 6.46512113e+00  ,6.78720329e-01],
 [ 9.22597602e+00  ,6.16120382e-01],
 [ 8.73178693e+00  ,1.21675692e+00],
 [ 1.04295135e+01  ,2.64727023e-01],
 [ 1.08479145e+01  ,6.48364453e-01],
 [ 1.27861035e+01  ,2.25565332e+00],
 [ 1.33666467e+01  ,1.44472730e+00],
 [ 1.32670676e+01  ,1.40899418e+00],
 [ 1.56292281e+01  ,2.59784641e+00],
 [ 1.43972646e+01  ,3.52892412e+00],
 [ 1.61782945e+01  ,5.50000153e+00],
 [ 1.72279170e+01  ,5.79448559e+00],
 [ 1.67738246e+01  ,4.58925503e+00],
 [ 1.78965750e+01  ,5.59895200e+00],
 [ 1.99576281e+01  ,6.40904357e+00],
 [ 1.93373902e+01  ,5.06221109e+00],
 [ 2.12985418e+01  ,7.40705331e+00],
 [ 2.11447677e+01  ,6.78257643e+00],
 [ 2.29460026e+01  ,7.33222946e+00],
 [ 2.35907149e+01  ,1.02653128e+01],
 [ 2.38527926e+01  ,1.11925295e+01],
 [ 2.48308974e+01  ,1.20914844e+01],
 [ 2.50328430e+01  ,1.37519524e+01],
 [ 2.49602050e+01  ,1.26177878e+01],
 [ 2.60194144e+01  ,1.29780208e+01],
 [ 2.61506726e+01  ,1.29622346e+01],
 [ 2.71699298e+01  ,1.33125132e+01],
 [ 2.73155709e+01  ,1.71428285e+01],
 [ 2.97607030e+01  ,1.76497498e+01],
 [ 2.96916657e+01  ,1.86884647e+01],
 [ 3.09549380e+01  ,1.59970051e+01],
 [ 3.08214757e+01  ,1.96254823e+01],
 [ 2.92117310e+01  ,1.97058296e+01],
 [ 3.15675222e+01  ,2.25455597e+01],
 [ 3.09357131e+01  ,2.20334136e+01],
 [ 3.09327338e+01  ,2.49463446e+01],
 [ 3.11274384e+01  ,2.45323236e+01],
 [ 3.28852535e+01  ,2.43160575e+01],
 [ 3.27513576e+01  ,2.48368764e+01],
 [ 3.22748680e+01  ,2.62016156e+01],
 [ 3.22873464e+01  ,2.66460814e+01],
 [ 3.18769508e+01  ,2.87018925e+01],
 [ 3.23274447e+01  ,3.14752979e+01],
 [ 3.15429138e+01  ,3.06687718e+01],
 [ 3.20386416e+01  ,3.15688870e+01],
 [ 3.12025495e+01  ,3.16982969e+01],
 [ 3.23349406e+01  ,3.42816498e+01],
 [ 3.26184761e+01  ,3.41715961e+01],
 [ 3.21508249e+01  ,3.68030095e+01],
 [ 3.16396028e+01  ,3.85900268e+01],
 [ 3.11703325e+01  ,3.87908864e+01],
 [ 3.03584143e+01  ,3.91500403e+01],
 [ 3.21356521e+01  ,3.90378759e+01],
 [ 2.89209884e+01  ,4.24137816e+01],
 [ 3.00260830e+01  ,4.17305617e+01],
 [ 3.04510136e+01  ,4.49032586e+01],
 [ 2.95811763e+01  ,4.24313982e+01],
 [ 2.96440160e+01  ,4.35390540e+01],
 [ 3.06212328e+01  ,4.51098151e+01],
 [ 2.87261643e+01  ,4.93965645e+01],
 [ 2.84620823e+01  ,4.69884386e+01],
 [ 2.76665447e+01  ,4.83550231e+01],
 [ 2.64088226e+01  ,5.00402354e+01],
 [ 2.60009509e+01  ,4.99092635e+01],
 [ 2.73745078e+01  ,5.17422644e+01],
 [ 2.51744186e+01  ,5.36429596e+01],
 [ 2.44297688e+01  ,5.36944534e+01],
 [ 2.46263408e+01  ,5.37056374e+01],
 [ 2.37992171e+01  ,5.43264148e+01],
 [ 2.28617888e+01  ,5.43086215e+01],
 [ 2.37359285e+01  ,5.59742299e+01],
 [ 2.10613192e+01  ,5.44697836e+01],
 [ 1.99362149e+01  ,5.55922207e+01],
 [ 1.89248947e+01  ,5.95943700e+01],
 [ 2.08342531e+01  ,5.73791437e+01],
 [ 1.91706412e+01  ,5.78478238e+01],
 [ 1.90643754e+01  ,6.06721042e+01],
 [ 1.66449383e+01  ,5.89089601e+01],
 [ 1.45380620e+01  ,5.97182591e+01],
 [ 1.46999033e+01  ,5.79719365e+01],
 [ 1.32888058e+01  ,5.95700788e+01],
 [ 1.19378036e+01  ,6.05410728e+01],
 [ 1.24467047e+01  ,6.19756606e+01],
 [ 1.19472189e+01  ,6.34839459e+01],
 [ 9.55383648e+00  ,6.12525432e+01],
 [ 7.97375785e+00  ,6.38732287e+01],
 [ 8.60902744e+00  ,6.33786593e+01],
 [ 7.82425846e+00  ,6.49511839e+01],
 [ 5.21346203e+00  ,6.36928418e+01],
 [ 3.84921726e+00  ,6.28972740e+01],
 [ 1.28649773e+00  ,6.25691212e+01],
 [ 4.38255701e+00  ,6.39084129e+01],
 [ 6.05261996e-01  ,6.27014136e+01],
 [ 1.91585608e+00  ,6.35914084e+01],
 [-2.00627125e-01  ,6.48160529e+01],
 [-2.26568861e+00  ,6.14984227e+01],
 [-4.40656164e+00  ,6.20588956e+01],
 [-1.87009248e+00  ,6.30700783e+01],
 [-2.68132534e+00  ,6.27047517e+01],
 [-4.05327501e+00  ,6.23835748e+01],
 [-6.33066289e+00  ,6.34384427e+01],
 [-6.67798664e+00  ,6.23706630e+01],
 [-9.40737593e+00  ,6.24127549e+01],
 [-9.65092565e+00  ,6.18668943e+01],
 [-1.04287471e+01  ,6.26161157e+01],
 [-1.07678602e+01  ,6.14271722e+01],
 [-1.36855240e+01  ,6.18776627e+01],
 [-1.18655111e+01  ,6.08816330e+01],
 [-1.34099618e+01  ,6.09226671e+01],
 [-1.46257627e+01  ,6.01956803e+01],
 [-1.59809481e+01  ,6.08558662e+01],
 [-1.71217478e+01  ,6.03804033e+01],
 [-1.80249498e+01  ,5.73039388e+01],
 [-1.91601118e+01  ,5.94260940e+01],
 [-1.86752358e+01  ,5.70346305e+01],
 [-1.91768897e+01  ,5.54040493e+01],
 [-2.12091201e+01  ,5.65726918e+01],
 [-2.02542279e+01  ,5.47280111e+01],
 [-2.26843376e+01  ,5.44443591e+01],
 [-2.05577604e+01  ,5.36687238e+01],
 [-2.35452896e+01  ,5.33807147e+01],
 [-2.30444170e+01  ,5.43033589e+01],
 [-2.38508854e+01  ,4.89932857e+01],
 [-2.62735028e+01  ,5.17075148e+01],
 [-2.59187565e+01  ,4.91499091e+01],
 [-2.78544378e+01  ,4.90641261e+01],
 [-2.60429068e+01  ,4.80189038e+01],
 [-2.98641348e+01  ,4.77297437e+01],
 [-2.72329526e+01  ,4.80114908e+01],
 [-2.95599484e+01  ,4.54259921e+01],
 [-2.79686844e+01  ,4.64192634e+01],
 [-3.00006191e+01  ,4.39169761e+01],
 [-2.90473775e+01  ,4.20578877e+01],
 [-2.92525328e+01  ,4.32953464e+01],
 [-2.99973659e+01  ,3.99216788e+01],
 [-2.98822389e+01  ,4.03811319e+01],
 [-2.98443892e+01  ,4.05971641e+01],
 [-3.05951280e+01  ,3.75987948e+01],
 [-3.09662447e+01  ,3.64639405e+01],
 [-3.14741481e+01  ,3.68087964e+01],
 [-3.28553231e+01  ,3.56517734e+01],
 [-3.33028751e+01  ,3.25899479e+01],
 [-3.19274037e+01  ,3.28136732e+01],
 [-3.11367285e+01  ,3.25380345e+01],
 [-3.20790195e+01  ,3.20096591e+01],
 [-3.33351767e+01  ,2.92925312e+01],
 [-3.25463247e+01  ,2.97541572e+01],
 [-3.10791733e+01  ,2.76652961e+01],
 [-3.09268376e+01  ,2.73216795e+01],
 [-2.99867815e+01  ,2.40676400e+01],
 [-3.00663590e+01  ,2.47275701e+01],
 [-3.11609300e+01  ,2.39004819e+01],
 [-3.15261475e+01  ,2.32165646e+01],
 [-3.08837662e+01  ,2.13913381e+01],
 [-2.90953258e+01  ,2.20107937e+01],
 [-2.78848445e+01  ,2.14814415e+01],
 [-2.92680284e+01  ,1.80211692e+01],
 [-2.74094665e+01  ,2.06610128e+01],
 [-2.77581956e+01  ,1.72194696e+01],
 [-2.82500251e+01  ,1.60742218e+01],
 [-2.70808624e+01  ,1.60999376e+01],
 [-2.49393297e+01  ,1.50348503e+01],
 [-2.61340256e+01  ,1.42598926e+01],
 [-2.41711021e+01  ,1.34911433e+01],
 [-2.50786651e+01  ,1.23983155e+01],
 [-2.51265467e+01  ,1.24741003e+01],
 [-2.35450690e+01  ,1.28085804e+01],
 [-2.33245295e+01  ,1.08307420e+01],
 [-2.27512700e+01  ,1.29657799e+01],
 [-2.19495118e+01  ,8.67005543e+00],
 [-2.01463580e+01  ,8.73472883e+00],
 [-2.02783466e+01  ,6.75866449e+00],
 [-1.95539724e+01  ,7.78320682e+00],
 [-1.71327642e+01  ,6.03481605e+00],
 [-1.79274492e+01  ,4.91702795e+00],
 [-1.66799288e+01  ,4.25954341e+00],
 [-1.59767195e+01  ,4.55962446e+00],
 [-1.68964556e+01  ,3.41140403e+00],
 [-1.35236069e+01 , 5.65302251e+00],
 [-1.45116400e+01 , 2.32064238e+00],
 [-1.37265394e+01  ,1.57080151e+00],
 [-1.19652761e+01  ,4.95403080e+00],
 [-1.04431784e+01  ,1.35360930e+00],
 [-1.05608754e+01  ,7.35815250e-01],
 [-7.83078036e+00  ,3.24566785e+00],
 [-6.70806125e+00  ,6.11820826e-01],
 [-7.45474373e+00  ,3.95807547e-01],
 [-5.81751912e+00 ,-1.08674704e+00],
 [-5.93332975e+00 ,-7.91406401e-01],
 [-5.15650899e+00 ,-1.42411567e+00],
 [-4.27069247e+00 , 4.87129565e-01],
 [-1.01172003e+00 , 1.10913476e+00],
 [-2.39369659e+00 ,-1.17983336e-03],
 [ 3.73882500e-01 , 1.02578216e-02]],np.float32)     

Observed_Track_2=np.array([[ 1.19856140e+00 ,-6.58280852e-01],
 [ 1.74332552e+00, -6.60347185e-01],
 [ 3.03333932e+00, -1.40082103e-01],
 [ 6.19625744e+00, -3.25757157e-01],
 [ 5.50692933e+00,  7.81552419e-01],
 [ 4.28770753e+00,  3.89867007e+00],
 [ 8.58104555e+00 ,-1.31679601e+00],
 [ 8.55633194e+00 , 8.69297952e-01],
 [ 9.76768858e+00  ,1.79030752e-01],
 [ 9.04004233e+00  ,5.96912994e-01],
 [ 1.00974366e+01  ,4.19938878e-01],
 [ 1.14049781e+01  ,2.14432023e+00],
 [ 1.27838995e+01  ,3.59534752e+00],
 [ 1.34170428e+01  ,1.44582043e+00],
 [ 1.31920510e+01  ,3.57293982e+00],
 [ 1.63795133e+01  ,3.28797968e+00],
 [ 1.48593020e+01  ,5.03211112e+00],
 [ 1.67716872e+01  ,3.79690152e+00],
 [ 1.78086472e+01  ,5.87403509e+00],
 [ 1.74524828e+01  ,5.93215608e+00],
 [ 1.98814234e+01  ,6.19141611e+00],
 [ 2.31537099e+01  ,6.61003474e+00],
 [ 2.16854140e+01  ,8.29796838e+00],
 [ 2.21403399e+01  ,1.05876196e+01],
 [ 2.20483290e+01  ,8.92221268e+00],
 [ 2.49529068e+01  ,9.13574879e+00],
 [ 2.25902182e+01  ,1.15170828e+01],
 [ 2.27303384e+01  ,1.14472350e+01],
 [ 2.42710055e+01  ,1.27503950e+01],
 [ 2.73806586e+01  ,1.34681942e+01],
 [ 2.72449089e+01  ,1.25046382e+01],
 [ 2.75308841e+01 , 1.58982393e+01],
 [ 2.72258466e+01 , 1.63077794e+01],
 [ 2.82892042e+01 , 1.75620957e+01],
 [ 2.81168892e+01 , 1.64636394e+01],
 [ 3.00566207e+01 , 1.90424130e+01],
 [ 2.89791346e+01 , 1.61073844e+01],
 [ 2.98857839e+01 , 1.99134455e+01],
 [ 3.10438234e+01 , 2.02204698e+01],
 [ 3.14799098e+01 , 2.13194039e+01],
 [ 3.00406955e+01 , 2.32992522e+01],
 [ 3.17472697e+01 , 2.38975697e+01],
 [ 3.03314324e+01 , 2.46079819e+01],
 [ 3.31280818e+01 , 2.61565049e+01],
 [ 3.36630952e+01 , 2.55125719e+01],
 [ 3.11774309e+01 , 2.77774855e+01],
 [ 3.16673743e+01 , 2.90586620e+01],
 [ 3.37121402e+01 , 3.25671684e+01],
 [ 3.07357041e+01 , 3.20196471e+01],
 [ 3.29373012e+01 , 3.18945740e+01],
 [ 3.23994206e+01 , 3.18867921e+01],
 [ 3.28001955e+01 , 3.44017357e+01],
 [ 3.28765354e+01 , 3.44664435e+01],
 [ 3.29112365e+01  ,3.48018386e+01],
 [ 3.24090444e+01  ,3.62821757e+01],
 [ 3.35695497e+01  ,3.78096133e+01],
 [ 3.16437614e+01  ,3.89562021e+01],
 [ 3.09954357e+01  ,3.77731523e+01],
 [ 3.09493402e+01  ,4.12718664e+01],
 [ 3.07310437e+01  ,4.08779641e+01],
 [ 3.01843872e+01  ,4.15244933e+01],
 [ 3.20017353e+01  ,4.44563536e+01],
 [ 2.90824669e+01  ,4.50399511e+01],
 [ 3.11965281e+01  ,4.57077790e+01],
 [ 3.08324575e+01  ,4.50766785e+01],
 [ 2.67001126e+01  ,4.68893935e+01],
 [ 2.81217679e+01  ,4.69455504e+01],
 [ 2.70742590e+01  ,4.62524685e+01],
 [ 2.92596310e+01  ,4.89358305e+01],
 [ 2.67461959e+01  ,5.07657832e+01],
 [ 2.54169946e+01  ,5.01934040e+01],
 [ 2.47759970e+01  ,5.18971181e+01],
 [ 2.49419935e+01  ,5.35336036e+01],
 [ 2.39469513e+01  ,5.32224755e+01],
 [ 2.40918229e+01  ,5.42040635e+01],
 [ 2.21995075e+01  ,5.56712082e+01],
 [ 2.03810723e+01  ,5.52245729e+01],
 [ 2.27507337e+01  ,5.51858195e+01],
 [ 2.17564538e+01  ,5.52888871e+01],
 [ 1.98069012e+01  ,5.71657563e+01],
 [ 1.94537561e+01  ,5.95236719e+01],
 [ 1.82928283e+01  ,5.82208991e+01],
 [ 1.78389992e+01  ,5.68124443e+01],
 [ 1.60695130e+01  ,5.89868214e+01],
 [ 1.50172554e+01  ,5.96784212e+01],
 [ 1.46835674e+01  ,6.09379653e+01],
 [ 1.43836057e+01  ,5.97192729e+01],
 [ 1.25802059e+01  ,6.07002046e+01],
 [ 1.07229234e+01  ,6.21919795e+01],
 [ 1.06429570e+01  ,6.20954871e+01],
 [ 9.02369008e+00  ,6.25530160e+01],
 [ 7.71905259e+00  ,6.30733896e+01],
 [ 7.99558556e+00  ,6.24829306e+01],
 [ 8.28605870e+00  ,6.21231017e+01],
 [ 5.76852334e+00  ,6.31252490e+01],
 [ 4.91824905e+00  ,6.32258428e+01],
 [ 4.30645575e+00  ,6.50434304e+01],
 [ 1.58505423e+00  ,6.42852123e+01],
 [ 3.13838286e+00  ,6.38818512e+01],
 [ 8.03065776e-02  ,6.30242474e+01],
 [-3.51396884e-02  ,6.31723081e+01],
 [-1.50833051e+00  ,6.36343852e+01],
 [-2.67982467e+00  ,6.47736356e+01],
 [-4.90277013e+00  ,6.32210559e+01],
 [-5.46276144e+00  ,6.19261500e+01],
 [-4.79665424e+00  ,6.39121027e+01],
 [-4.78961859e+00  ,6.20995512e+01],
 [-5.23644686e+00  ,6.30287300e+01],
 [-6.86012274e+00  ,6.17722759e+01],
 [-8.88981832e+00  ,6.06131012e+01],
 [-1.08539818e+01  ,6.26679772e+01],
 [-1.00524618e+01  ,6.29828551e+01],
 [-1.31826355e+01  ,6.09868643e+01],
 [-1.12658535e+01  ,6.04827470e+01],
 [-1.40740036e+01  ,6.12341377e+01],
 [-1.39595441e+01  ,5.87186989e+01],
 [-1.38563369e+01  ,5.93393179e+01],
 [-1.56367637e+01  ,6.05052094e+01],
 [-1.75363503e+01  ,5.70896818e+01],
 [-1.79964257e+01  ,5.77057343e+01],
 [-1.74565798e+01  ,5.76650712e+01],
 [-1.96196793e+01  ,5.68313501e+01],
 [-1.93769888e+01  ,5.61933268e+01],
 [-2.23707099e+01  ,5.53670885e+01],
 [-2.06090963e+01  ,5.42555212e+01],
 [-2.32830090e+01  ,5.38013817e+01],
 [-2.36492534e+01  ,5.29793317e+01],
 [-2.45607791e+01  ,5.38847648e+01],
 [-2.51161549e+01  ,5.16431415e+01],
 [-2.38720349e+01  ,5.01720926e+01],
 [-2.39969985e+01  ,4.87950246e+01],
 [-2.64891066e+01  ,5.04654043e+01],
 [-2.44412709e+01  ,4.81002189e+01],
 [-2.64107623e+01  ,4.86684241e+01],
 [-2.84322168e+01  ,4.66103236e+01],
 [-2.92522832e+01  ,4.45466521e+01],
 [-2.85771108e+01  ,4.67202412e+01],
 [-2.71440821e+01  ,4.43235398e+01],
 [-2.99882881e+01  ,4.49106793e+01],
 [-2.98906909e+01  ,4.19817549e+01],
 [-2.97455546e+01  ,4.14249948e+01],
 [-3.01695342e+01  ,4.00072774e+01],
 [-3.11752374e+01  ,3.87076131e+01],
 [-3.19600991e+01  ,3.82801523e+01],
 [-3.13211713e+01  ,3.80810727e+01],
 [-3.09068268e+01  ,3.53686427e+01],
 [-3.08406473e+01  ,3.49578977e+01],
 [-2.98793749e+01  ,3.40588489e+01],
 [-3.16924887e+01  ,3.27541215e+01],
 [-3.21454153e+01  ,3.08949799e+01],
 [-3.15758599e+01  ,3.05140508e+01],
 [-3.27223817e+01  ,3.06643349e+01],
 [-3.03341230e+01  ,2.93355280e+01],
 [-3.10457093e+01  ,3.03594627e+01],
 [-3.01915343e+01  ,2.68841280e+01],
 [-3.25155638e+01  ,2.36735746e+01],
 [-3.12660334e+01  ,2.66985212e+01],
 [-2.79030585e+01  ,2.57971412e+01],
 [-2.93506006e+01  ,2.31168582e+01],
 [-3.00422797e+01  ,2.24898492e+01],
 [-3.06567007e+01  ,2.23885946e+01],
 [-2.84260237e+01  ,1.81680475e+01],
 [-3.00380114e+01  ,2.13522479e+01],
 [-2.86444453e+01  ,2.00416391e+01],
 [-2.98456287e+01  ,1.65152176e+01],
 [-2.50101737e+01  ,1.63208840e+01],
 [-2.65119201e+01  ,1.61326835e+01],
 [-2.81168682e+01  ,1.41141214e+01],
 [-2.71339689e+01  ,1.41311360e+01],
 [-2.53357401e+01  ,1.43008737e+01],
 [-2.54309893e+01  ,1.36707932e+01],
 [-2.40694589e+01  ,1.44027919e+01],
 [-2.47604793e+01  ,1.17760962e+01],
 [-2.33706123e+01  ,8.74691596e+00],
 [-2.28799247e+01  ,1.06483856e+01],
 [-2.14941312e+01  ,8.19417467e+00],
 [-2.04880962e+01  ,8.41559964e+00],
 [-1.96135059e+01  ,8.33743337e+00],
 [-1.83215601e+01  ,9.12018024e+00],
 [-1.90077065e+01  ,7.06883581e+00],
 [-1.65738663e+01  ,5.75165502e+00],
 [-1.76330379e+01  ,3.79501811e+00],
 [-1.53540771e+01  ,5.35024497e+00],
 [-1.40665427e+01  ,3.19820721e+00],
 [-1.57271960e+01  ,3.29261803e+00],
 [-1.38532443e+01  ,2.16024628e+00],
 [-1.35135887e+01  ,2.82470538e+00],
 [-1.21301613e+01  ,2.70937785e+00],
 [-1.11925792e+01  ,1.70311771e+00],
 [-8.60883957e+00  ,1.68578413e+00],
 [-7.79556124e+00  ,1.31244396e+00],
 [-6.41598469e+00  ,2.27675956e+00],
 [-6.47127431e+00  ,8.99135111e-01],
 [-6.85811890e+00  ,4.41418188e-01],
 [-4.12335865e+00 ,-3.61469041e-01],
 [-4.54223882e+00 ,-6.86389287e-01],
 [-3.97107804e+00 , 1.23332325e+00],
 [-1.58995405e+00 , 2.03368814e-01],
 [-2.25559203e+00 ,-8.66699419e-02],
 [ 3.77451100e-01 ,-5.74012423e-01]],np.float32)

Observed_Track_3=np.array([[  1.58053494   ,0.42751485],
 [  2.22375311  ,-1.07351424],
 [  2.86794626  , 0.81194233],
 [  4.83861416  , 0.72959362],
 [  3.33322574  , 1.68973909],
 [  5.85823885  , 1.96591221],
 [  6.78485379  ,-1.28247723],
 [  7.16382258  , 1.15471163],
 [  9.79786864  ,-0.07204717],
 [  7.58682671  , 1.4110842 ],
 [ 11.44182673  , 1.6632196 ],
 [ 11.51031947  , 2.38348937],
 [ 13.64233178  , 2.3363648 ],
 [ 14.2775933   , 2.68956146],
 [ 13.52799621  , 3.71188415],
 [ 15.17630729  , 4.91494616],
 [ 15.60687599  , 3.38070712],
 [ 17.67091235  , 5.54839191],
 [ 18.60528774  , 4.24401473],
 [ 17.77432561  , 5.36305352],
 [ 20.15378965  , 6.88961008],
 [ 21.30311935  , 6.80743437],
 [ 21.13782149  , 8.17337802],
 [ 21.88725758  , 8.01762152],
 [ 23.86016411  , 8.51558895],
 [ 24.87979684  , 8.69101515],
 [ 24.30051933  , 9.93577154],
 [ 24.60119578  , 9.21104226],
 [ 23.42774263  ,13.76064903],
 [ 26.25170866  ,14.23909824],
 [ 27.75508379  ,13.39953948],
 [ 26.54614608  ,13.75717461],
 [ 27.30216749  ,18.3628256 ],
 [ 26.22056769  ,15.42217495],
 [ 29.64622578  ,15.40035724],
 [ 27.55578119  ,20.09091191],
 [ 28.96746565  ,18.65254154],
 [ 31.31441776  ,18.76926862],
 [ 28.86585121  ,20.23469108],
 [ 29.70667599  ,22.87478992],
 [ 29.03071163  ,20.86437115],
 [ 32.14977474  ,21.85146271],
 [ 32.28653323  ,23.85888608],
 [ 29.88846656  ,26.73253947],
 [ 31.93223865  ,26.41046698],
 [ 32.28099941  ,27.69038918],
 [ 33.78523427  ,28.42443212],
 [ 32.16475928  ,29.51517073],
 [ 31.16346848  ,29.57094106],
 [ 32.63566164  ,30.40045455],
 [ 34.5635303   ,30.52886563],
 [ 32.2969099   ,35.64666252],
 [ 33.48409535  ,32.75445454],
 [ 32.18195842  ,35.07613336],
 [ 30.57212757  ,35.88193116],
 [ 32.22583577  ,36.47546374],
 [ 32.6577485   ,38.41200284],
 [ 31.97985866  ,38.96085968],
 [ 32.71578546  ,40.45066203],
 [ 30.94093518  ,40.38398772],
 [ 30.37596003  ,43.06414777],
 [ 31.50211149  ,43.43097804],
 [ 30.44244218  ,41.92293564],
 [ 29.05031904  ,44.04048306],
 [ 29.19310532  ,46.50113507],
 [ 29.70748534  ,45.87138155],
 [ 28.25602965  ,47.66971287],
 [ 28.12537203  ,48.07492728],
 [ 26.8135442   ,48.56309931],
 [ 25.34881699  ,49.84263994],
 [ 24.35316972  ,51.67327076],
 [ 25.76169044  ,50.66634149],
 [ 24.63540816  ,52.16030594],
 [ 24.23317348  ,53.60808228],
 [ 23.4730526   ,52.76192297],
 [ 23.17001307  ,55.57005996],
 [ 20.87775027  ,56.23429256],
 [ 21.67127528  ,55.1956114 ],
 [ 20.65787795  ,56.67538636],
 [ 20.05844707  ,56.052286  ],
 [ 19.12973052  ,57.14648008],
 [ 17.43325848  ,58.35508244],
 [ 17.4080406   ,60.73251699],
 [ 16.66439015  ,58.85800893],
 [ 16.29668088  ,60.7653804 ],
 [ 12.13287153  ,60.01488866],
 [ 13.41961043  ,59.8589092 ],
 [ 13.50549644  ,61.44107551],
 [ 13.20572814  ,62.14269372],
 [ 12.70571489  ,60.22713463],
 [ 10.04994316  ,61.85182675],
 [  7.73503121  ,62.21899598],
 [  6.68105704  ,62.09680566],
 [  6.12128912  ,62.36486314],
 [  7.3036956   ,61.82736196],
 [  4.79706567  ,64.24591078],
 [  3.30684145  ,62.74793756],
 [  4.05869799  ,63.59819238],
 [  1.7963288   ,63.74936971],
 [  1.93580048  ,61.73596027],
 [ -0.72068665  ,63.92981616],
 [ -1.80180866  ,63.05372218],
 [ -1.7969803   ,64.93228033],
 [ -2.06320098  ,64.82506488],
 [ -3.58055506  ,63.4444714 ],
 [ -4.71333493  ,63.19052343],
 [ -6.68608811  ,61.5091174 ],
 [ -7.8126079   ,62.87480651],
 [ -7.0268197   ,62.02678314],
 [ -8.36269743  ,63.08712844],
 [-10.81777081  ,62.43578056],
 [-10.09433251  ,60.80258856],
 [-12.2234415   ,59.83016193],
 [-12.72478422  ,62.05878235],
 [-12.2698361   ,59.82565149],
 [-13.48670161  ,57.75970134],
 [-15.0706834   ,60.0935702 ],
 [-15.07586283  ,58.15330309],
 [-16.15424307  ,58.90581549],
 [-18.76955134  ,56.70690936],
 [-19.71254291  ,55.92990465],
 [-21.07193477  ,57.13920387],
 [-20.56459097  ,57.77159139],
 [-23.28185314  ,56.69357705],
 [-20.93130904  ,54.83359417],
 [-21.36962732  ,54.32725452],
 [-22.86719538  ,53.70550138],
 [-24.87998905  ,51.69871082],
 [-25.48312235  ,51.32199381],
 [-23.87054995  ,51.50936732],
 [-26.66810022  ,50.09353107],
 [-26.03023574  ,48.64006189],
 [-27.12494239  ,47.51607758],
 [-26.24066463 , 46.94869146],
 [-28.91681709 , 47.61709787],
 [-27.57246629 , 44.85273119],
 [-30.12154879 , 44.69474306],
 [-28.28199489 , 40.9058615 ],
 [-30.16185556 , 43.15390645],
 [-31.48308273 , 43.06461266],
 [-30.08120363 , 42.79599386],
 [-30.52918234 , 40.13064929],
 [-33.05143804 , 38.21625668],
 [-30.73078623 , 37.75463638],
 [-30.79111401 , 37.1076457 ],
 [-32.09321793 , 36.32734883],
 [-31.6782976  , 36.02271353],
 [-30.90927961 , 34.74155971],
 [-30.3986391  , 33.70056415],
 [-30.87158428 , 31.54900037],
 [-31.00238315 , 28.75512615],
 [-29.94699968 , 31.44967159],
 [-29.51007615 , 28.56285051],
 [-28.9994905  , 28.15821188],
 [-31.77011801 , 27.0092893 ],
 [-30.64157947 , 25.71829609],
 [-30.39552112 , 26.13070183],
 [-31.45392544 , 23.75351565],
 [-30.25503052 , 24.13751843],
 [-31.14764235 , 21.05331128],
 [-29.16047848 , 23.12059726],
 [-28.4782311  , 19.66777182],
 [-28.65939125 , 18.18708583],
 [-29.58698703 , 19.73298071],
 [-28.04851553 , 18.14942469],
 [-27.01491145 , 17.58158755],
 [-26.46897692 , 14.49591733],
 [-25.32799139 , 16.38224398],
 [-26.08660846 , 17.40240706],
 [-25.17616542 , 12.46170484],
 [-25.61301825 , 13.59418497],
 [-24.28852153 , 12.90611972],
 [-24.03237673 , 12.24230125],
 [-21.59079666 ,  9.69370992],
 [-22.58250803 , 10.68665665],
 [-22.03314835 ,  8.09864125],
 [-22.22335434 ,  8.80031121],
 [-21.08579186 ,  9.46341342],
 [-19.33603643 ,  8.01819589],
 [-16.66992737 ,  6.1680921 ],
 [-16.64700396 ,  6.7251469 ],
 [-16.53663955 ,  6.95217582],
 [-17.7462007  ,  4.84470257],
 [-16.44773209 ,  3.77892667],
 [-13.71018301 ,  4.12410244],
 [-13.79245186 ,  2.27469376],
 [-12.84001382 ,  1.14736729],
 [-11.38132483 ,  3.2590303 ],
 [ -9.51368738 ,  0.54922329],
 [-11.0758419  ,  1.39799232],
 [ -7.35812404 ,  2.6849619 ],
 [ -7.54258212 ,  0.30943238],
 [ -6.87131357 ,  1.6173813 ],
 [ -6.70962221 ,  1.36524642],
 [ -5.47242291 , -0.12934254],
 [ -3.87589338 ,  1.17018343],
 [ -3.52095554 ,  0.72079414],
 [ -0.86391266 , -1.08396182],
 [ -1.98911795 , -0.52446348],
 [ -1.0957921  ,  0.52639719]],)

for i in (Observed_Track_1,Observed_Track_2,Observed_Track_3):
        
        
        draw_Kalman_p(true_track,i)  #position

        draw_Kalman_V(true_track,i)  #velocity

        draw_Kalman_a(true_track,i)  #acceleration
      
       