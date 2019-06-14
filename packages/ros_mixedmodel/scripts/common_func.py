import numpy as np

def VCStoWCS(OGX, OGY, OGHeading, PVX, PVY):
    MAT = np.column_stack((PVX, PVY))

    c = np.cos(OGHeading)
    s = np.sin(OGHeading)

    ROT = np.matrix([[c,-s],[s,c]])

    tmp_global = np.matmul(ROT, MAT.transpose())
    tmp_global[0,:] += OGX
    tmp_global[1,:] += OGY

    return(tmp_global.transpose())

def WCStoVCS(OGX, OGY, OGHeading, PGX, PGY):
    MAT = np.column_stack((PGX-OGX,PGY-OGY))

    c = np.cos(OGHeading)
    s = np.sin(OGHeading)

    ROT = np.matrix([[c,s], [-s,c]])

    tmp_vcs = np.matmul(ROT, MAT.transpose())

    return(tmp_vcs.transpose())
