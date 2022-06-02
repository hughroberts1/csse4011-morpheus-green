####################################################################################################
# Machine Learning module used to classify data retrieved from the BLE Mesh setup in the
# CSSE4011 final project to determine whether it will rain tomorrow.
#
# Author: Oliver Roman
# Date: 01/06/2022
####################################################################################################

from sklearn.neighbors import KNeighborsClassifier
from sklearn.model_selection import train_test_split
import pandas as pd
 
df = pd.read_csv('weather_data.csv')
# Add column for if it will rain tomorrow
rain_tmr = []
for item in df["mm of Rain"]:
        if item > 0:
                rain_tmr.append(1)
        else:
                rain_tmr.append(0)

df["Rain Tomorrow"] = rain_tmr

print(df.head())
def knn_train():
        
        X = df[features]
        y = df['Rain Tomorrow']
        X_train, X_test, y_train, y_test = train_test_split(X, y, random_state=42)

        model = KNeighborsClassifier(n_neighbors=2)
        model.fit(X_train, y_train)
        model.score(X_test, Y_test)
        return model.predict(X_test)