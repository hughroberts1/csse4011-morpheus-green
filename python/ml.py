####################################################################################################
# Machine Learning module used to classify data retrieved from the BLE Mesh setup in the
# CSSE4011 final project to determine whether it will rain tomorrow.
#
# Author: Oliver Roman
# Date: 01/06/2022
####################################################################################################

from sklearn.neighbors import KNeighborsClassifier
from sklearn.ensemble import RandomForestClassifier
from sklearn.model_selection import train_test_split
from sklearn import metrics
import pandas as pd
import matplotlib.pyplot as plt
import webscraper
# Trains KNN with full training set and takes sensor readings X_in to give a prediction y_predit
# as to whether it will rain tomorrow
def knn_train_realtime(X_in):
        
        X_train = df[features]
        y_train = df['Rain Tomorrow']

        X_test = X_in
        #y_test = y_in # this would be tomorrow's data on whether it rained or not
        model = KNeighborsClassifier(n_neighbors=20)
        model.fit(X_train, y_train)
        y_predict = model.predict(X_test)
        print("Accuracy:", model.score(X_test, y_test))

def rf_train_realtime(X_in):
        X_train = df[features]
        y_train = df['Rain Tomorrow']

        X_test = X_in
        #y_test = y_in # this would be tomorrow's data on whether it rained or not
        model = RandomForestClassifier(n_estimators=500)
        model.fit(X_train, y_train)
        y_predict = model.predict(X_test)
        print("Accuracy:", metrics.accuracy_score(y_test, y_predict))


df = pd.read_csv('weather_data.csv')
# Add column for if it will rain tomorrow
rain_tmr = []
for item in df["mm of Rain"]:
        if item > 0:
                rain_tmr.append(1)
        else:
                rain_tmr.append(0)
# Get rid of first element to shift the array to the left
rain_tmr.pop(0)
# Introduce a null value for the last element since we don't know if it will rain tmr
rain_tmr.append(0)

df["Rain Tomorrow"] = rain_tmr

df = df.fillna(0, axis = "columns")

features = ["Avg Temperature", "Max Temperature", "Min Temperature",\
            "Avg Humidity", "Max Humidity", "Min Humidity",\
            "Avg Pressure", "Max Pressure", "Min Pressure","mm of Rain"]
# Changing the types of all data columns to be a numerical type
df[["Avg Temperature", "Max Temperature", "Min Temperature", "Avg Humidity", "Avg Pressure",\
    "mm of Rain", "Rain Tomorrow"]]\
= df[["Avg Temperature", "Max Temperature", "Min Temperature", "Avg Humidity", "Avg Pressure",\
      "mm of Rain", "Rain Tomorrow"]].apply(pd.to_numeric)

X = df[features]
y = df['Rain Tomorrow']


# Main loop if this file is opened to show optimisation metrics
if __name__ == "__main__":
        X_train, X_test, y_train, y_test = train_test_split(X, y, random_state=42)
        
        model = KNeighborsClassifier(n_neighbors=20)
        model.fit(X_train, y_train)
        y_predict = model.predict(X_test)
        print(model.score(X_test, y_test))
        scores = []
        for n in range(1,40):
                model = KNeighborsClassifier(n_neighbors=n)
                model.fit(X_train,y_train)
                scores.append(model.score(X_test,y_test))

        model2 = RandomForestClassifier(n_estimators=500)
        model2.fit(X_train, y_train)
        y_predict = model2.predict(X_test)
        print("Accuracy:",metrics.accuracy_score(y_test, y_predict))
        feature_imp = pd.Series(model2.feature_importances_,index=features)
        print(feature_imp)

        plt.figure()
        plt.plot(range(1,40), scores)
        plt.title("KNN Optimisation")
        plt.xlabel("Number of Neighbours")
        plt.ylabel("Score ratio")
        plt.show()

        # So based off this we can estimate a neighbour pick of around 20 is appropriate

        plt.figure()
        df['Rain Tomorrow'].hist(align=('mid'))

        plt.title("Days raining vs not raining")
        # We do get creative
        plt.xlabel("0: Not Raining                                                      1: Raining")
        plt.ylabel("Days")
        plt.legend()
        plt.show()