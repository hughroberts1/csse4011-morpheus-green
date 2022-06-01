from sklearn.neighbors import KNeighborsClassifier 
import pandas as pd
 

def get_training_model():

        training
        model = KNeighborsClassifier(n_neighbors=2)
        model.fit(rssi_train,position_train)
        return model

model = get_trainingmodel()
model.predict()
