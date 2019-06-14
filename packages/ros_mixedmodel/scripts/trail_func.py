import numpy as np
import pandas as pd
import statsmodels.formula.api as smf
from common_func import *
import rospy

#   import rpy2
#   import rpy2.robjects as robjects
#   from rpy2.robjects.packages import importr
#   from rpy2.robjects import FloatVector, IntVector, StrVector

#   # import R's "base", "utils", "stats", and "nlme" packages
#   base = importr('base')
#   #utils = importr('utils')
#   #stats = importr('stats')
#   nlme = importr('nlme') # library('nlme')

def chopping(database,host_msg,RARE_MAX):
    tmp = WCStoVCS(host_msg.world_x, host_msg.world_y, host_msg.heading, database[['world_x']], database[['world_y']])
    idx = np.where(tmp[:,0] < 0)[0]
    drop_idx = np.where(np.sqrt(np.array(tmp[idx,0])**2+np.array(tmp[idx,1])**2) > RARE_MAX)[0]
    idx2 = np.where(tmp[:,0] > 0)[0]
    drop_idx2 = np.where(np.sqrt(np.array(tmp[idx2,0])**2+np.array(tmp[idx2,1])**2) > 250)[0]

    database = database.drop(database.index[idx[np.concatenate((drop_idx,drop_idx2))]])

    return(database)

def fit_road_shape_model(database):
    md = smf.mixedlm(formula = "vcs_y ~ 1+vcs_x+I(vcs_x**2)+I(vcs_x**3)", data=database, groups=database["ID"])
    road_shape_model = md.fit()

    return(road_shape_model)

def estimate_road_shape(road_shape_model, database):
    # get unique trail vcs x position
    vcs_x = np.unique(database[['vcs_x']].values)
    
    # create a DataFrame
    newx = pd.DataFrame({'vcs_x':vcs_x})
    
    # predict the vcs y position by road shape model
    vcs_y = road_shape_model.predict(newx).values

    # combine the result and store in a list
    est_road_shape = {"vcs_x": vcs_x, "vcs_y": vcs_y}

    return(est_road_shape)

#   def fit_road_shape_model(database):
#       vid = StrVector(database[['ID']].values)
#       vcs_x = FloatVector(database[['vcs_x']].values)
#       vcs_y = FloatVector(database[['vcs_y']].values)

#       # assign data to R data.frame
#       df = {'VID':vid, 'VCS_X': vcs_x, 'VCS_Y': vcs_y}
#       Trails = robjects.DataFrame(df)

#       # fixs <- formul(VCS_Y ~ poly(VCS_X,3))
#       fixs = robjects.Formula('VCS_Y ~ poly(VCS_X,3)')

#       # ranefs <- formula(~1|VID)
#       ranefs = robjects.Formula('~1|VID')

#       # fit_road_shape <- lme(fixed=fixs, data=Trails, random=ranefs)
#       road_shape_model = nlme.lme(fixed=fixs, data=Trails, random=ranefs)

#       return(road_shape_model)


#   def estimate_road_shape(road_shape_model, database):
#       # get unique trail vcs x position
#       vcs_x = np.unique(database[['vcs_x']].values)
#       
#       # convert vcs x into R format
#       vcs_x_R = FloatVector(vcs_x)
#       
#       # create an R DataFrame
#       df = {'VCS_X':vcs_x_R}
#       newx = robjects.DataFrame(df)
#       
#       # predict the vcs y position by road shape model
#       vcs_y = robjects.r.predict(road_shape_model, newdata=newx, level=0)
#       
#       # combine the result and store in a list
#       pred_road_shape = {"vcs_x": vcs_x, "vcs_y": vcs_y}

#       return(pred_road_shape)
