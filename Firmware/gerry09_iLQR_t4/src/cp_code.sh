dest="/Users/gerry/Dropbox (GaTech)/Painting/Graffiti_media+data/2022-02-14_lqg_klaus/"$1
echo $dest
mkdir "$dest"
cp controllers/controller_simple.h "$dest"
cp controllers/controller_lqg.h "$dest"
cp state_estimators/state_estimator_kf.h "$dest"
cp communication/debug.h "$dest"
