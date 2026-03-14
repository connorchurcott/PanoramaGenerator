
function [gray, color] = prepareImage(image)
    image = imread(image);
    gray = rgb2gray(im2double(imresize(image, [750 NaN])));
    color = im2double(imresize(image, [750 NaN]));
end

function [iKernels] = createImpulseKernels()
    iKernels = zeros(7, 7, 16);

    iKernels(7, 4, 1) = 1;
    iKernels(7, 5, 2) = 1; 
    iKernels(6, 2, 3) = 1;
    iKernels(5, 1, 4) = 1;
    iKernels(4, 1, 5) = 1;
    iKernels(3, 1, 6) = 1;
    iKernels(2, 2, 7) = 1;
    iKernels(1, 5, 8) = 1;
    iKernels(1, 4, 9) = 1;
    iKernels(1, 3, 10) = 1;
    iKernels(2, 6, 11) = 1;
    iKernels(5, 7, 12) = 1;
    iKernels(4, 7, 13) = 1;
    iKernels(3, 7, 14) = 1;
    iKernels(6, 6, 15) = 1;
    iKernels(7, 3, 16) = 1;
end

function [valuesCount] = countCircularValues(values, n)
    
    loopedValues = cat(3, values, values);

    for i = 1:16
        valuesCount(:, :, i) = sum(loopedValues(:, :, i:i+(n-1)), 3);
    end

end

function [scores] = scoreFunction(image, iKernels)
    
    for i = 1:16
        shiftedImages(:, :, i) = imfilter(image, iKernels(:, :, i));
    end

    scores = zeros(size(image));
    for i = 1:16
        scores = scores + abs(image - shiftedImages(:, :, i));
    end

end


function [seperatedCorners] = my_fast_detector(image)
    n = 12; 
    t = 0.1; 

    % impulse kernel to created shifted images, 1 for each pixel of circle
    % use the def from opencv docs to calculate light/dark
    iKernels = createImpulseKernels();
    for i = 1:16
        shiftedImages(:, :, i) = imfilter(image, iKernels(:, :, i));
        brighterPixels(:, :, i) = shiftedImages(:, :, i) > image + t;
        darkerPixels(:, :, i) = shiftedImages(:, :, i) < image - t;
    end

    % count up amount of contiguous pixels in the circle
    brightCount = countCircularValues(brighterPixels, n);
    darkCount = countCircularValues(darkerPixels, n);
    corners = any(brightCount == n, 3) | any(darkCount == n, 3);
    
    % non-maxima seperation as stated in the opencv docs
    scores = scoreFunction(image, iKernels);
    localmax = imdilate(scores, ones(3));
    seperatedCorners = corners & (scores == localmax);
end

function [fastr] = harrisCornerness(image, fastCorners)
    threshold = 0.001;

    sobx = [-1 0 1; -2 0 2; -1 0 1];
    gaus = fspecial('gaussian', 5, 1);
    dog = conv2(gaus, sobx);

    ix = imfilter(image, dog);
    iy = imfilter(image, dog');
    ix2 = imfilter(ix .* ix, gaus);
    iy2 = imfilter(iy .* iy, gaus);
    ixiy = imfilter(ix .* iy, gaus); 

    harcor = (ix2 .* iy2 - ixiy .* ixiy) - 0.05*(ix2 + iy2).^2;

    harcor = harcor .* fastCorners;
    fastr = harcor > threshold;

end

function [fastFeat, fastVal] = pointDescription(image, fast)
    % i decdied to use this workflow: orb -> extractFeatures -> matchFeatures ->
    % showMatchedFeatures

    [rowFast, colFast] = find(fast);
    locationsFast = [colFast, rowFast];

    fastORB = ORBPoints(locationsFast);

    [featuresFast, validCornersFast] = extractFeatures(image, fastORB);

    fastFeat = featuresFast;
    fastVal = validCornersFast;
end

function [match1, match2] = matchingWork(fastFeat1, fastVal1, fastFeat2, fastVal2)
    
    indexsFast = matchFeatures(fastFeat1, fastFeat2); 

    match1 = fastVal1(indexsFast(:, 1), :);
    match2 = fastVal2(indexsFast(:, 2), :);

end

function panorama = ransacWork(image1, image2, match1, match2)
    % based off of the matlab panorama generation tutorial posted
    % didn't bother trying to implement the for loops or dynamic stuff
    % because it was too hard and I didnt have enough time. 
    
    confidenceFast = 99;
    maxTrialsFast = 100;

    confidenceFastr = 99; 
    maxTrialsFastr = 200;

    %og Fastr (confidence = 99.9), (maxTrialsFastr = 1000)

    tform = estgeotform2d(match2, match1, ...
        "projective", Confidence=confidenceFastr, MaxNumTrials=maxTrialsFastr);
    
    panoramaWidth = 480 * 2; 
    panorama = zeros([750 panoramaWidth 3], "like", image1); 
    panoramaView = imref2d([750 panoramaWidth], [0 480*2], [0 750]);

    panorama(1:size(image1,1), 1:size(image1,2), :) = image1; 

    warpedIm = imwarp(image2, tform, OutputView=panoramaView);
    mask = imwarp(true(size(image2, 1), size(image2, 2)), tform, OutputView=panoramaView);

    for color = 1:3
        panorama(:, :, color) = imblend(warpedIm(:, :, color), panorama(:, :, color), mask, ForegroundOpacity=1);
    end

    %imshow(panorama);

end


function generateImages()
    % initial images
    [S1_im1, S1i1c] = prepareImage("S1-im1.png");
    [S1_im2, S1i2c] = prepareImage("S1-im2.png");

    [S2_im1, S2i1c] = prepareImage("S2-im1.jpg");
    [S2_im2, S2i2c] = prepareImage("S2-im2.jpg");

    [S3_im1, S3i1c] = prepareImage("S3-im1.jpg");
    [S3_im2, S3i2c] = prepareImage("S3-im2.jpg");

    [S4_im1, S4i1c] = prepareImage("S4-im2.jpg");
    [S4_im2, S4i2c] = prepareImage("S4-im3.jpg");

    % part 1
    fastPointsS1 = my_fast_detector(S1_im1);
    corvisS1 = S1i1c;
    corvisS1(fastPointsS1 > 0) = 1;
    imwrite(corvisS1, "S1-fast.png");

    fastPointsS2 = my_fast_detector(S2_im1);
    corvisS2 = S2i1c;
    corvisS2(fastPointsS2 > 0) = 1;
    imwrite(corvisS2, "S2-fast.png");

    % part 2
    fastrPointsS1 = harrisCornerness(S1_im1, fastPointsS1);
    corvisS1R = S1i1c; 
    corvisS1R(fastrPointsS1 > 0) = 1;
    imwrite(corvisS1R, "S1-fastR.png")

    fastrPointsS2 = harrisCornerness(S2_im1, fastPointsS2);
    corvisS2R = S2i1c;
    corvisS2R(fastrPointsS2 > 0) = 1;
    imwrite(corvisS2R, "S2-fastR.png");


    % part 3
    [fFeatS1_im1, fValS1_im1] = pointDescription(S1_im1, fastPointsS1);
    fastPointsS1i2 = my_fast_detector(S1_im2);
    [fFeatS1_im2, fValS1_im2] = pointDescription(S1_im2, fastPointsS1i2);
    [matchFastS1i1, matchFastS1i2] = matchingWork(fFeatS1_im1, fValS1_im1, fFeatS1_im2, fValS1_im2);
    showMatchedFeatures(S1i1c, S1i2c, matchFastS1i1, matchFastS1i2);
    window = getframe(gca);
    visual = frame2im(window);
    imwrite(visual, "S1-fastMatch.png");

    [frFeatS1_im1, frValS1_im1] = pointDescription(S1_im1, fastrPointsS1);
    fastrPointsS1i2 = harrisCornerness(S1_im2, fastPointsS1i2);
    [frFeatS1_im2, frValS1_im2] = pointDescription(S1_im2, fastrPointsS1i2);
    [matchFastrS1i1, matchFastrS1i2] = matchingWork(frFeatS1_im1, frValS1_im1, frFeatS1_im2, frValS1_im2);
    showMatchedFeatures(S1i1c, S1i2c, matchFastrS1i1, matchFastrS1i2);
    window = getframe(gca);
    visual = frame2im(window);
    imwrite(visual, "S1-fastrMatch.png");

    [fFeatS2_im1, fValS2_im1] = pointDescription(S2_im1, fastPointsS2);
    fastPointsS2i2 = my_fast_detector(S2_im2);
    [fFeatS2_im2, fValS2_im2] = pointDescription(S2_im2, fastPointsS2i2);
    [matchFastS2i1, matchFastS2i2] = matchingWork(fFeatS2_im1, fValS2_im1, fFeatS2_im2, fValS2_im2);
    showMatchedFeatures(S2i1c, S2i2c, matchFastS2i1, matchFastS2i2);
    window = getframe(gca);
    visual = frame2im(window);
    imwrite(visual, "S2-fastMatch.png");

    [frFeatS2_im1, frValS2_im1] = pointDescription(S2_im1, fastrPointsS2);
    fastrPointsS2i2 = harrisCornerness(S2_im2, fastPointsS2i2);
    [frFeatS2_im2, frValS2_im2] = pointDescription(S2_im2, fastrPointsS2i2);
    [matchFastrS2i1, matchFastrS2i2] = matchingWork(frFeatS2_im1, frValS2_im1, frFeatS2_im2, frValS2_im2);
    showMatchedFeatures(S2i1c, S2i2c, matchFastrS2i1, matchFastrS2i2);
    window = getframe(gca);
    visual = frame2im(window);
    imwrite(visual, "S2-fastrMatch.png");


    % part 4
    S1pan = ransacWork(S1i1c, S1i2c, matchFastrS1i1, matchFastrS1i2);
    imwrite(S1pan, "S1-panorama.png");


    S2pan = ransacWork(S2i1c, S2i2c, matchFastrS2i1, matchFastrS2i2);
    imwrite(S2pan, "S2-panorama.png");

    fastPointsS3 = my_fast_detector(S3_im1);
    fastrPointsS3 = harrisCornerness(S3_im1, fastPointsS3);

    fastPointsS3i2 = my_fast_detector(S3_im2);
    fastrPointsS3i2 = harrisCornerness(S3_im2, fastPointsS3i2);

    [frFeatS3_im1, frValS3_im1] = pointDescription(S3_im1, fastrPointsS3);
    [frFeatS3_im2, frValS3_im2] = pointDescription(S3_im2, fastrPointsS3i2);
    [matchFastrS3i1, matchFastrS3i2] = matchingWork(frFeatS3_im1, frValS3_im1, frFeatS3_im2, frValS3_im2);
    S4pan = ransacWork(S3i1c, S3i2c, matchFastrS3i1, matchFastrS3i2);
    imwrite(S4pan, "S3-panorama.png");


    fastPointsS4 = my_fast_detector(S4_im1);
    fastrPointsS4 = harrisCornerness(S4_im1, fastPointsS4);

    fastPointsS4i2 = my_fast_detector(S4_im2);
    fastrPointsS4i2 = harrisCornerness(S4_im2, fastPointsS4i2);

    [frFeatS4_im1, frValS4_im1] = pointDescription(S4_im1, fastrPointsS4);
    [frFeatS4_im2, frValS4_im2] = pointDescription(S4_im2, fastrPointsS4i2);
    [matchFastrS4i1, matchFastrS4i2] = matchingWork(frFeatS4_im1, frValS4_im1, frFeatS4_im2, frValS4_im2);
    S4pan = ransacWork(S4i1c, S4i2c, matchFastrS4i1, matchFastrS4i2);
    imwrite(S4pan, "S4-panorama.png");





end

generateImages();


[image, imagec] = prepareImage("S1-im1.png");
fastPoints = my_fast_detector(image);
fastrPoints = harrisCornerness(image, fastPoints);
[fastFeat, fastVal] = pointDescription(image, fastPoints);
[fastrFeat, fastrVal] = pointDescription(image, fastrPoints);


[image2, image2c] = prepareImage("S1-im2.png");
fastPoints2 = my_fast_detector(image2);
fastrPoints2 = harrisCornerness(image2, fastPoints2);
[fastFeat2, fastVal2] = pointDescription(image2, fastPoints2);
[fastrFeat2, fastrVal2] = pointDescription(image2, fastrPoints2);

[matchFast1, matchFast2] = matchingWork(fastFeat, fastVal, fastFeat2, fastVal2);
panorama = ransacWork(imagec, image2c, matchFast1, matchFast2);
%[matchFastr1, matchFastr2] = matchingWork(fastrFeat, fastrVal, fastrFeat2, fastrVal2); 
%panorama = ransacWork(imagec, image2c, matchFastr1, matchFastr2);
imshow(panorama);

corvis = image; 
corvis(fastrPoints > 0) = 1;
%imshow([corvis fastrPoints fastrPoints2]);
