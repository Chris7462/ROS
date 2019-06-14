import time
import statsmodels.api as sm
import statsmodels.formula.api as smf

data = sm.datasets.get_rdataset("dietox", "geepack").data
md = smf.mixedlm(formula = "Weight ~ 1+Time+I(Time**2)+I(Time**3)", data=data, groups=data["Pig"])

for i in range(0,1,1):
    t = time.time()
    mdf = md.fit()
    elapsed1 = time.time() - t
    start_params = mdf.params.values #np.append(mdf.params.values)
    t = time.time()
    mdf2 = md.fit(start_params) #, do_cg=False)
    elapsed2 = time.time() - t
    print([elapsed1, elapsed2])


mdf.summary()


















# predict
time = np.unique(data['Time'].values)
# create a DataFrame
newx = pd.DataFrame({'Time':time})

y = mdf.predict(newx).values

    # predict the vcs y position by road shape model
    road_shape_mode.predict(newx)


# R
library('nlme')
library('geepack')
data(dietox)

fit <- lme(Weight~Time, random=~1|Pig, data=dietox)
summary(fit)


  pred_road_shape <- list(Global_X=NULL, Global_Y=NULL, VCS_X=NULL,VCS_Y=NULL)
  pred_road_shape$VCS_X <- sort(unique(trails$VCS_X))
  newx <- data.frame(VCS_X=pred_road_shape$VCS_X)
  #pred_road_shape$VCS_Y <- as.numeric(predict(road_shape_model, newx, level=0)-predict(road_shape_model,data.frame(VCS_X=0),level=0))
  pred_road_shape$VCS_Y <- as.numeric(predict(road_shape_model, newx, level=0))

