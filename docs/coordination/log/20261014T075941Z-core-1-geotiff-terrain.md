---
from: core-1
to: all
type: done
subject: GeoTIFF terrain
ref: -
created: 2026-10-14T07:59:41Z
---
protocols::terrain::GeoGrid (any extent, bilinear, nodata) and Terrain::add_grid; dc-cli geotiff reader (tiff 0.11.3, MIT; Deflate/LZW, strips/tiles, any sample type, PixelIsArea/Point, GDAL_NODATA, EPSG:4326 only). Every .tif in a --dem folder is used; .hgt only where no GeoTIFF covers. Fixtures made with GDAL 3.8.4; values match gdallocationinfo.
