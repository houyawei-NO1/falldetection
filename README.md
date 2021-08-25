# falldetection

##PYTHON数组到c++数组 
dataIn = dataIn[:headerLength]
从a[0]复制到a[headerLength-1]
a = a.left(headerLength)

dataIn = dataIn[headerLength:]
从a[headerLength]复制到末尾
a = a.mid(headerLength) 

dataIn = dataIn[1:]
从a[1]复制到末尾
a = a.mid(1)

##python for range

range(1,10) 是 1-9 :
>>> range(1,10)
[1, 2, 3, 4, 5, 6, 7, 8, 9]

range(10) 是 0-9 :
>>> range(10)
[0, 1, 2, 3, 4, 5, 6, 7, 8, 9]


https://github.com/dpilger26/NumCpp
##NumPy					NumCpp
a[2, 3]					a(2, 3)
a[2:5, 5:8]				a(nc::Slice(2, 5), nc::Slice(5, 8))
						a({2, 5}, {5, 8})
a[:, 7]					a(a.rSlice(), 7)
a[a > 5]				a[a > 0]
a[a > 5] = 0			a.putMask(a > 5, 0)