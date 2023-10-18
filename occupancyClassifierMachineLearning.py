from sklearn.model_selection import train_test_split
from sklearn.neighbors import KNeighborsClassifier
from sklearn.preprocessing import MinMaxScaler
from sklearn.metrics import confusion_matrix, classification_report, accuracy_score, roc_curve, auc
from sklearn.ensemble import RandomForestClassifier
from sklearn.preprocessing import LabelEncoder
import keras
from keras.utils import np_utils
from keras.models import Sequential
from keras.layers import Dense
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
df_1_noAct=pd.read_csv(r'C:\\Users\\jason\\Desktop\\Occupancy Dataset\BMECCSHM(office 1 people no activity).csv', sep=",")
df_2_noAct=pd.read_csv(r'C:\\Users\\jason\\Desktop\\Occupancy Dataset\BMECCSHM(office 2 people no activity).csv', sep=",")
df_3_noAct=pd.read_csv(r'C:\\Users\\jason\\Desktop\\Occupancy Dataset\BMECCSHM(office 3 people no activity).csv', sep=",")
df_2_Act=pd.read_csv(r'C:\\Users\\jason\\Desktop\\Occupancy Dataset\BMECCSHM(office 2 people activity).csv', sep=",")
df_3_Act=pd.read_csv(r'C:\\Users\\jason\\Desktop\\Occupancy Dataset\BMECCSHM(office 3 people activity).csv', sep=",")
df_0_noAct=pd.read_csv(r'C:\\Users\\jason\\Desktop\\Occupancy Dataset\BMECCSHM(office no people activity).csv', sep=",")
df=pd.concat([df_0_noAct, df_1_noAct, df_2_noAct, df_2_Act, df_3_noAct, df_3_Act]).reset_index(drop=True)
scaler = MinMaxScaler()

columns = ['CO2', 'TVOC', 'Temperature', 'Humidity', 'Pressure', 'Gas_Res', 'Lux', 'PM25']
df[columns]=scaler.fit_transform(np.array(df[columns]))
plt.figure(figsize=(10,10))
plt.title('Box Plot for Features', fontdict={'fontsize':18})
ax=sns.boxplot(data=df.drop(['Time', 'Activity ', 'Occupancy'], axis=1),orient="h")
plt.figure(figsize=(10,8))
plt.title('Correlation of Data', fontdict={'fontsize':18})
sns.heatmap(df.corr(), annot=True, linewidths=0.1)
b, t=plt.ylim()
b+=0.5
t-=0.5
plt.ylim(b,t)
ax= sns.pairplot(df[['CO2', 'TVOC', 'Temperature', 'Humidity', 'Pressure', 'Gas_Res', 'Lux', 'PM25', 'Activity ', 'Occupancy']], hue='Occupancy', diag_kind='hist', palette='Set2')
#KNN neutral network
X=df[['CO2', 'TVOC', 'Temperature', 'Humidity', 'Pressure', 'Gas_Res', 'Lux', 'PM25', 'Activity ']]
y=df['Occupancy']
X_train, X_test,y_train, y_test=train_test_split(X, y, test_size =0.2, random_state=42)
modelKNN = KNeighborsClassifier(n_neighbors=4)
modelKNN.fit(X_train, y_train)
knn_preds=modelKNN.predict(X_test)
print(classification_report(y_test, knn_preds))
print(accuracy_score(y_test, knn_preds))
plt.figure(figsize=(10,10))
plt.title("KNN Confusion Matrix")
ax = sns.heatmap(confusion_matrix(y_test,knn_preds), annot = True, fmt = "d")
b, t = plt.ylim()
b += 0.5
t -= 0.5
plt.ylim(b,t)
accuracy_list=[]
n_list=[]
for n in range(1,100):
    n_list.append(n)
    modelKNN = KNeighborsClassifier(n_neighbors=n)
    modelKNN.fit(X_train, y_train)
    knn_preds = modelKNN.predict(X_test)

    score = accuracy_score(y_test, knn_preds)

    accuracy_list.append(score)

print(max(accuracy_list))
print(n_list[accuracy_list.index(max(accuracy_list))])
plt.figure()
plt.plot(n_list, accuracy_list)


# ANN neural network
encoder = LabelEncoder()
encoder.fit(y)
encoded_y=encoder.transform(y)
catg_y = np_utils.to_categorical(encoded_y)
X_train, X_test,y_train, y_test=train_test_split(X, catg_y, test_size =0.2, random_state=42)
modelANN = Sequential()
modelANN.add(Dense(18, input_dim=9, activation='relu'))
modelANN.add(Dense(18, activation='relu'))
modelANN.add(Dense(4, activation='softmax'))

modelANN.compile(loss='binary_crossentropy', optimizer='adam', metrics=['accuracy'])

modelANN.fit(X_train, y_train, epochs=100, batch_size=10)
_, accuracy = modelANN.evaluate(X_test, y_test)
print('Accuracy: %.5f'%(accuracy*100))

#random forest neural network
# rf_accuracy_list=[]
# rf_n_list=[]
# classifier=RandomForestClassifier(n_estimators=100)
# classifier.fit(X_train, y_train)
#
# rf_preds = classifier.predict(X_test)
#
# accuracy_score(y_test, rf_preds)
#
# for n in range(1,100):
#     rf_n_list.append(n)
#     classifier= RandomForestClassifier(n_estimators=n)
#     classifier.fit(X_train, y_train)
#     rf_preds = classifier.predict(X_test)
#
#     rf_score = accuracy_score(y_test, rf_preds)
#
#     rf_accuracy_list.append(rf_score)
#
# print(max(rf_accuracy_list))
# print(rf_n_list[rf_accuracy_list.index(max(rf_accuracy_list))])

# plt.figure()
# plt.plot(rf_n_list, rf_accuracy_list)
plt.show()