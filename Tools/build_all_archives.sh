#!/bin/sh

make archives BOARDS=hbrain-v00
make archives BOARDS=hbrain-v00-bmp280
make archives BOARDS=hbrain-v10
make archives BOARDS=vrbrain-v45

make archives BOARDS=vrbrain-v51

make archives BOARDS=vrbrain-v51Pro

make archives BOARDS=vrbrain-v52

make archives BOARDS=vrbrain-v52Pro

make archives BOARDS=vrubrain-v51

make archives BOARDS=vrubrain-v52

make distclean
