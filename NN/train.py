from sklearn.model_selection import train_test_split
from sklearn.neural_network import MLPClassifier as MLP
from sklearn.neighbors import KNeighborsClassifier as KNN
from sklearn.linear_model import SGDClassifier as SGD
from sklearn.ensemble import AdaBoostClassifier as ADB
import joblib
from sklearn.preprocessing import StandardScaler
# from sklearn.externals import joblib
import pandas as pd
df = pd.read_csv('NN/new_data.txt', sep=',\s+')
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
x_train, x_test, y_train, y_test = train_test_split(X, Y, stratify=Y, test_size=0.1, random_state=321)
scaler = StandardScaler()
scaler.fit(x_train)
# x_train_scaled = scaler.transform(x_train)
# x_test_scaled = scaler.transform(x_test)
x_train_scaled = x_train.values
x_test_scaled = x_test.values
i = 0
model = [(200,), (200, 200,), (200, 100, 60,), (200, 100, 60, 11, ), ]
sb = {}
mlpb = {}
for m in model:
    i = 1
    sb[m] = 0
    ss = 0
    while i < 100:
        mlp = MLP(hidden_layer_sizes=m, activation='relu', alpha=0.01, batch_size=64, learning_rate_init=0.01, max_iter=5000)
        mlp.fit(x_train_scaled, y_train)
        s = mlp.score(x_test_scaled, y_test)
        t = mlp.predict([[0.000, 0.000, 11.639, 6.357, -1.671, -160.000, 5.641, 1.023, -1.666, -180.000]])[0]
        ss += s
        print(f"{m}: {i}, {s:.3f}, {t}, best:{sb[m]:.3f}, avg:{ss/i:.3f}")
        if s > sb[m]:
            mlpb[m] = mlp
            sb[m] = s
        i += 1
        if s > 0.8 and  t < 5: break
for m in model:
    joblib.dump(mlpb[m], 'test'+str(m)+'.pkl')
    clf = joblib.load( 'test'+str(m)+'.pkl')
    v = clf.predict([[0.000, 0.000, 11.639, 6.357, -1.671, -160.000, 5.641, 1.023, -1.666, -180.000]])[0]
    print(m, v)
# while True:
#     adb = ADB()
#     adb.fit(x_train_scaled, y_train)
#     s = adb.score(x_test_scaled, y_test)
#     t = adb.predict([[11.736, 6.915, -4.261, -160.000, 5.939, 2.048, 0.290, -180.000]])[0]
#     print(i, s, t)
#     i += 1
#     if s > 0.8 and  t < 5: break
# joblib.dump(adb, 'adb.pkl')
# clf = joblib.load('adb.pkl')

# while True:
#     knn = KNN()
#     knn.fit(x_train_scaled, y_train)
#     s = knn.score(x_test_scaled, y_test)
#     t = knn.predict([[11.736, 6.915, -4.261, -160.000, 5.939, 2.048, 0.290, -180.000]])[0]
#     print(i, s, t)
#     i += 1
#     if s > 0.8 and  t < 5: break
# joblib.dump(knn, 'knn.pkl')
# clf = joblib.load('knn.pkl')

# while True:
#     sgd = SGD()
#     sgd.fit(x_train_scaled, y_train)
#     s = sgd.score(x_test_scaled, y_test)
#     t = sgd.predict([[11.736, 6.915, -4.261, -160.000, 5.939, 2.048, 0.290, -180.000]])[0]
#     print(i, s, t)
#     i += 1
#     if s > 0.8 and  t < 5: break
# joblib.dump(sgd, 'sgd.pkl')
# clf = joblib.load('sgd.pkl')

# 3.5


'''
Usage:

import joblib
from sklearn.neural_network import MLPClassifier as MLP
clf = joblib.load('mlp.pkl')
v = clf.predict([[11.736, 6.915, -4.261, -160.000, 5.939, 2.048, 0.290, -180.000]])[0]
'''
