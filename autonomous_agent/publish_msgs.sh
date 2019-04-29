#!/bin/zsh

rostopic pub /liability/infochan/eth/signing/offer robonomics_msgs/Offer "$(cat test_offer.yaml)" -1

# &&  rostopic pub /liability/infochan/eth/signing/demand robonomics_msgs/Demand "$(cat test_ask.yaml)" -1
