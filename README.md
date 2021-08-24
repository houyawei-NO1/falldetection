# falldetection
 
dataIn = dataIn[:headerLength]
从a[0]复制到a[headerLength-1]
a = a.left(headerLength)

dataIn = dataIn[headerLength:]
从a[headerLength]复制到末尾
a = a.mid(headerLength) 

dataIn = dataIn[1:]
从a[1]复制到末尾
a = a.mid(1)