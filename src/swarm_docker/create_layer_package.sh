

# $1 will be the repo name
# $2 will be the number of layers to include


RUN_DIR=$(pwd)

mkdir /tmp/swarm_project
cd /tmp/swarm_project || exit
mkdir all
mkdir pack

docker save "$1" > all.tar
tar -xf ./all.tar -C ./all
cd all || exit

# We are inside the (extracted) archive of the image; let's remove the layers we don't care about

LIST=$(jq '.[0].Layers' < manifest.json | grep "\".*/" -o)

echo "List:   $LIST"

declare -i i
declare -i N_FILES
declare -i N_REMOVE

N_FILES=$(find . -maxdepth 1 -mindepth 1 -type d | wc -l)
N_REMOVE=$((N_FILES-$2))
echo "Pruning $N_REMOVE layer(s) [$N_FILES total layers, $2 to be kept]"
i=0


for f in $LIST
do
  FILE="${f:1}"
  if ((i >= N_REMOVE)); then
    echo "Using layer $FILE in package"
    mv "$FILE"/
  fi
  i=$((i+1))
done

# mission accomplished, let's compress and leave

cd ..

tar -zcf ./all.tar.gz ./all
cp ./all.tar.gz "$RUN_DIR/image.tar.gz"
rm -r /tmp/swarm_project/ # clean up temp files so we don't bloat the system

#