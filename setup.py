import os

# compile jtlv compiler initially (build the java source with javac)
print "Compiling jtlv compiler..."
os.chdir('/src/etc/jtlv')
os.system('sh build.sh')
os.chdir('/../../..')