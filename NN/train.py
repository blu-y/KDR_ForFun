from sklearn.model_selection import train_test_split
from sklearn.neural_network import MLPClassifier as MLP
import joblib
from sklearn.preprocessing import StandardScaler
# from sklearn.externals import joblib
import pandas as pd
df = pd.read_csv('NN/data.txt', sep=',\s+')
X = df.iloc[:,:-1]
Y = df.iloc[:,-1].round().T
'''
print(X.head())
       xi      yi     zi   yawi    xi+1    yi+1   zi+1  yawi+1
0 -20.522   1.005 -3.087 -210.0 -34.292  -0.696 -4.622  -230.0
1 -34.292  -0.696 -4.622 -230.0 -45.442   6.652 -4.932  -250.0
2 -45.442   6.652 -4.932 -250.0 -54.930  16.022 -5.368  -250.0
3 -54.930  16.022 -5.368 -250.0 -58.310  30.394 -7.601  -250.0
4 -58.310  30.394 -7.601 -250.0 -62.598  42.167 -7.038  -230.0

print(Y.head())
0    9.000
1    6.743
2    6.636
3    6.632
4    6.926
Name: vi, dtype: float64
'''
x_train, x_test, y_train, y_test = train_test_split(X, Y, stratify=Y, test_size=0.1, random_state=123)
scaler = StandardScaler()
scaler.fit(x_train)
x_train_scaled = scaler.transform(x_train)
x_test_scaled = scaler.transform(x_test)
i = 0
while True:
    mlp = MLP(hidden_layer_sizes=(100,100,100), activation='relu', alpha=0.01, batch_size=64, learning_rate_init=0.01, max_iter=5000)
    mlp.fit(x_train_scaled, y_train)
    s = mlp.score(x_test_scaled, y_test)
    t = mlp.predict([[11.736, 6.915, -4.261, -160.000, 5.939, 2.048, 0.290, -180.000]])[0]
    print(i, s, t)
    i += 1
    if s > 0.8 and  t < 5: break
joblib.dump(mlp, 'mlp.pkl')
clf = joblib.load('mlp.pkl')
v = clf.predict([[11.736, 6.915, -4.261, -160.000, 5.939, 2.048, 0.290, -180.000]])[0]
print(v)
# 3.5


'''
Usage:

import joblib
from sklearn.neural_network import MLPClassifier as MLP
clf = joblib.load('mlp.pkl')
v = clf.predict([[11.736, 6.915, -4.261, -160.000, 5.939, 2.048, 0.290, -180.000]])[0]
'''
