FoamFile
{
    format      ascii;
    class       dictionary;
    object      decomposeParDict;
}

numberOfSubdomains 16;

method          scotch;   // good default, no manual tuning needed
