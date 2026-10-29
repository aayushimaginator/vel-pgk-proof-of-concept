FoamFile
{
    version     2.0;
    format      ascii;
    class       dictionary;
    object      surfaceSubsetDict;
}

// select faces from a set/zone written by surfaceCheck
faceSet         problemFaces;   // or the exact name surfaceCheck reported
