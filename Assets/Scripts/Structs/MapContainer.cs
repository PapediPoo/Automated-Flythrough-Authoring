using System;

namespace RSUtils
{
    public struct MapContainer
    {
        public RSGrid rsgrid;
        public bool[,,] bitmap;
        public float[,,] visibility;
        public float[,,] distancetransform;
        public float[,,] heightmap;
    }
}
