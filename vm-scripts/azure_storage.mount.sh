#!/usr/bin/env bash

# Copyright (c) 2018 Bosch Software Innovations GmbH.
# All rights reserved. This program and the accompanying materials
# are made available under the terms of the Eclipse Public License v2.0
# which accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v20.html
#
# Contributors:
#    Bosch Software Innovations GmbH - initial creation
#
# Disclaimer: This software is experimental and intended to be used on the Bosch Connected Experience 2018

AZURE_STORAGE_DIR=$HOME/azure-storage
CREDENTIALS_DIR=/shared/credentials
CREDENTIALS_FILE=$CREDENTIALS_DIR/azure_bcw18_data_storage
FILE_SHARE=data

mkdir -p $AZURE_STORAGE_DIR
mkdir -p $CREDENTIALS_DIR

if [[ ! -e $CREDENTIALS_FILE ]]; then
    cat <<- EOF > $CREDENTIALS_FILE
	username=#WILL_BE_PROVIDED_AT_BCW_2018#
	password=#WILL_BE_PROVIDED_AT_BCW_2018#
	EOF
fi

if grep -q '#WILL_BE_PROVIDED_AT_BCW_2018#' $CREDENTIALS_FILE; then
    echo "Please fill in the credentials in $CREDENTIALS_FILE."
    exit 1
fi

if ! mountpoint -q $AZURE_STORAGE_DIR; then
    sudo mount -t cifs \
        //bcw18.file.core.windows.net/$FILE_SHARE \
        $AZURE_STORAGE_DIR \
        -o uid=vagrant \
        -o gid=vagrant \
        -o vers=3.0 \
        -o dir_mode=0777 \
        -o file_mode=0777 \
        -o serverino \
        -o rw \
        -o credentials=$CREDENTIALS_FILE
fi
