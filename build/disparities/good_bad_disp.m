bad = imread('878.png');   %1316, 1393, 1411, 1427, 1428
good = imread('939.png');  %1317, 1376, 1413, 1429, 1431; 878 car 939
bad2=bad(101:620,161:1180);
good2=good(101:620,161:1180);

bad3 = ind2rgb(bad2,jet);
good3 = ind2rgb(good2,jet);

figure(1), imshow(bad3), title('bad');
figure(2), imshow(good3), title('good');

bad2d = cast(bad2,'double');
bad2d = bad2d(:);
bad2_var = var(bad2d)
bad2_std = sqrt(bad2_var);

good2d = cast(good2,'double');
good2d = good2d(:);
good2_var = sqrt(var(good2d))
good2_std = sqrt(good2_var);

% Reject anything above var 50
