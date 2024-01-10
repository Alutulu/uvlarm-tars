import cv2
import numpy as np
import matplotlib.pyplot as plt

def fig2data ( fig ):
    # If we haven't already shown or saved the plot, then we need to
    # draw the figure first...
    fig.canvas.draw()

    # Now we can save it to a numpy array.
    data = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep='') 
    data = data.reshape(fig.canvas.get_width_height()[::-1] + (3,))
    return data

def histo(im):
	im_hist = plt.figure()
	myplt   = im_hist.add_subplot ( 111 )

	myplt.hist(im.ravel(),256,[0,256]);

	histogramme=fig2data(im_hist)
	histogramme = cv2.cvtColor(histogramme, cv2.COLOR_BGR2GRAY)
	hh, wh = histogramme.shape
	him, wim = im.shape
	coeff=him/hh
	histogramme = cv2.resize(histogramme, (np.int(wh*coeff),800)) 
	imPlusHist=np.concatenate((im,histogramme), axis=1)
	return imPlusHist

def myplot(y,dotsx=[],dotsy=[]):
    x=range(0,len(y))
    fig = plt.figure()
    myplt   = fig.add_subplot ( 111 )
    myplt.plot(x,y) 
    #myplt.plot(range(0,len(x)),np.zeros(len(x))) 
    if(len(dotsx)!=0):
        myplt.plot(dotsx,dotsy, 'g^')
    #plt.show()
    return fig2data(fig) 
