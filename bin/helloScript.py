#!/usr/bin/env python3
# coding: utf-8


from tutorial_package import coucou


if __name__ == "__main__":
    try:
        coucou("my friend!")
    except KeyboardInterrupt:
        pass
    finally:
        pass
