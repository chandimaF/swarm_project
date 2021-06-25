export SIZETEST_NAME='sizeteste_16kb'
export SIZETEST_BASE_OS='alpine'

# Grab a fresh Ubuntu image (there's probably already one on the system) and
docker pull $SIZETEST_BASE_OS
docker run -td --name $SIZETEST_NAME $SIZETEST_BASE_OS

# Push the blank version to the registry
docker commit @SIZETEST_NAME localhost:5000/$SIZETEST_NAME
docker push localhost:5000/$SIZETEST_NAME

# Create and push the new version to the registry
docker exec $SIZETEST_NAME sh -c 'head -c 16384 /bin/busybox > test.txt'
docker commit $SIZETEST_NAME localhost:5000/$SIZETEST_NAME
docker push localhost:5000/$SIZETEST_NAME

echo "---------"
# Print the manifest
docker manifest inspect --insecure localhost:5000/$SIZETEST_NAME | grep size
