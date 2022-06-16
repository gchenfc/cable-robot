# dest="/Users/gerry/Dropbox (GaTech)/Painting/Graffiti_media+data/2022-02-24_lqg_klaus/"$1
dest="/Users/gerry/Dropbox (GaTech)/Painting/Graffiti_media+data/2022-02-25_gouttefarde_klaus/"$1
echo $dest
mkdir "$dest"
git rev-parse HEAD > $dest/COMMIT
cp controllers/controller_simple.h "$dest"
cp controllers/controller_gouttefarde.h "$dest"
cp communication/debug.h "$dest"
cp constants.h "$dest"
