import handlerSubsystem as hSub


### Test ParameterObject Class ###
print "Test ParameterObject Class"
print

# Test constructor and __repr__
print "Test constructor and __repr__"
para = hSub.ParameterObject()
print para
para.name = 'Jim'
para.type = 'float'
para.des = 'test'
para.default = 0.0
para.max = 10.0
para.min = 0.0
para.value = 2.0
print para
print

# Test Setter
print "Test setter"
para.setValue('test')
para.type = None
para.setValue('test')
