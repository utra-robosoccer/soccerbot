# count number of points in mesh.proto file
# we should have 50,000 or less points per file for fast rendering
# count number of ','

for file in ../protos/Soccerbot_meshes/*.proto
do
  ls "$file" >> results.out
  fgrep -o , "$file" | wc -l >> results.out
done

