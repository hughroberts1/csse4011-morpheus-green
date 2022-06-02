####################################################################################################
# Machine Learning module used to classify data retrieved from the BLE Mesh Embedded setup in the
# CSSE4011 final project to determine whether it will rain tomorrow.
#
# Author: Oliver Roman
# Date: 01/06/2022
####################################################################################################

from sklearn.neighbors import KNeighborsClassifier 
import pandas as pd
 
df = pd.read_csv('weather_data.csv')
df.head() 

def get_training_model():

        model = KNeighborsClassifier(n_neighbors=2)
        #model.fit(rssi_train,position_train)
        return model

#model = get_trainingmodel()
#model.predict()
