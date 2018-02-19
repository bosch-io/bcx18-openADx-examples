# Eclipse Hono Applications

Copyright (c) 2018 Bosch Software Innovations GmbH.
All rights reserved. This program and the accompanying materials
are made available under the terms of the Eclipse Public License v2.0
which accompanies this distribution, and is available at
http://www.eclipse.org/legal/epl-v20.html

Contributors: Bosch Software Innovations GmbH - initial creation

*Disclaimer: This software is experimental and intended to be used on the Bosch Connected Experience 2018*

The apps are subscribing to Hono's AMQP topics and perform some message processing.

## Setup

In the provided virtual machine the prerequisites are already met and the pip packages already installed.
The scripts can be run without further setup. There may be script-specific steps (see documentation in the scripts).

To run the scripts somewhere else have a look at the below prerequisites and installation.

### Prerequisites

- Python 2
- Pip
- a [virtual environment](http://docs.python-guide.org/en/latest/dev/virtualenvs/) (e.g. [virtualenv](https://virtualenv.pypa.io) or [conda](https://conda.io))


### Installation

```
$ virtualenv --python=/usr/bin/python2 .env
$ . .env/bin/activate
$ pip install -r requirements.txt
```
