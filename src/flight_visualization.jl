using DataFrames, PyPlot, PGFPlots

# To add:
# read in the policy too (or Q)
# plot the log data against the policy in some way
# directly compare the actions taken to the simulated actions required by the policy. Confirm it's doing the correct thing with the real data.
# make a movie of the policy as calculated from Q against the flight data states.
# compaer the fligth test data with the simulated data in some way

function getLogData(logFile::AbstractString, outFile::AbstractString)
# logFile="/home/emueller/ACAS/datafiles/FlightTest/CA2_results_191741.log.txt"

	logF = open(logFile)
	logO = open(outFile)

	fLines = readlines(logF)

	close(logF)
	close(logO)

	dat = Array(Float64, size(fLines,1), 17)
	params = Array(Float64,16)
	for (i,e) in enumerate(fLines)
		t, subs = split(e[1:end-2],":")
		#subs, dummy = split(subs,"\\")
		dat[i,1] = parse(t)
		s = split(subs,",")
		for (j,d) in enumerate(s)
			dat[i,j+1] = parse(d)
		end
	end

	# Now fill in the appropriate colums
	fDat = DataFrame()
	fDat[:time] = dat[:,1]
	fDat[:stepCounter] = dat[:,2]
	fDat[:actionInd] = dat[:,3]
	fDat[:rx] = dat[:,4]
	fDat[:ry] = dat[:,5]
	fDat[:vox] = dat[:,6]
	fDat[:voy] = dat[:,7]
	fDat[:vix] = dat[:,8]
	fDat[:viy] = dat[:,9]
	fDat[:dx] = dat[:,10]
	fDat[:dy] = dat[:,11]
	fDat[:xi] = dat[:,12]
	fDat[:yi] = dat[:,13]
	fDat[:xo] = dat[:,14]
	fDat[:yo] = dat[:,15]
	fDat[:xt] = dat[:,16]
	fDat[:yt] = dat[:,17]


	return fDat
end


