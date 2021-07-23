
for i in 1 2 4 8 16 32 64 128 512 1024 2048 4096 8192 16384 32768 65536 131072 262144 524288 1048576 2097152 4194304 8388608 16777216 33554432 67108864 134217728 268435456
do
  export SIZETEST_SIZE=$i
  export SIZETEST_NAME='sizeteste_'$SIZETEST_SIZE
  export SIZETEST_BASE_OS='alpine'

  docker tag localhost:5000/$SIZETEST_NAME $SIZETEST_NAME
#
#  # Grab a fresh Ubuntu image (there's probably already one on the system) and
#  docker pull $SIZETEST_BASE_OS
#  docker run -td --name $SIZETEST_NAME $SIZETEST_BASE_OS
#
#  # Push the blank version to the registry
#  docker commit $SIZETEST_NAME localhost:5000/$SIZETEST_NAME
#  docker push localhost:5000/$SIZETEST_NAME
#
#  # Create and push the new version to the registry
#  docker exec $SIZETEST_NAME sh -c "head -c ${SIZETEST_SIZE} /dev/urandom > test.txt"
#  docker commit $SIZETEST_NAME localhost:5000/$SIZETEST_NAME
#  docker push localhost:5000/$SIZETEST_NAME
#
#  echo "---------"
#  # Print the manifest
#  docker manifest inspect --insecure localhost:5000/$SIZETEST_NAME | grep size
done


