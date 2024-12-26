clear;
clc;

constellation = [ -0.707-0.707j,  -1+0j,  0+1j,  -0.707+0.707j, ...
                  0-1j,  0.707-0.707j,  0.707+0.707j,  1+0j];

pattern_index = 100;
load H_and_R.mat;

%% initialization
rng(8000);
test_channel =501;
Hmatrix = zeros(test_channel, 4, 4);
for i=1:test_channel
    real_part = normrnd(0, 0.5^(1/2), [4, 4]);
    imag_part = normrnd(0, 0.5^(1/2), [4, 4]);
    Hmatrix(i, :, :) = real_part + 1j*imag_part;
    if (i == pattern_index)
        Hmatrix(i, :, :) = H;
    end
end

numSymbols = 160;
SNRdB = -4:1:25;
bitsPerSymbol = 3;

bitData = randi([0 1], numSymbols * bitsPerSymbol, 1)';
Tx_Symbols = zeros(1, numSymbols);
for i=1:numSymbols
    index = bitData(bitsPerSymbol*i) + 2*bitData(bitsPerSymbol*i-1) + 4*bitData(bitsPerSymbol*i-2) + 1;
    Tx_Symbols(i) = constellation(index);
end
Tx_Symbols = reshape(Tx_Symbols,4, []);

Es = mean(abs(Tx_Symbols).^2, "all");

Rx_Symbols = zeros(test_channel, length(SNRdB), size(Tx_Symbols, 1), size(Tx_Symbols, 2));
for h = 1:test_channel
    for k = 1:length(SNRdB)
        SNR = 10^(SNRdB(k)/10); % Linear SNR
        No = Es / (2 * SNR); % Noise Energy per dimension
        noise = normrnd(0, sqrt(No), size(Tx_Symbols)) + 1j * normrnd(0, sqrt(No), size(Tx_Symbols));
        % noise = sqrt(No) * (randn(size(Tx_Symbols)) + 1j * randn(size(Tx_Symbols)));
        Rx_Symbols(h, k, :, :) = squeeze(Hmatrix(h, :, :)) * Tx_Symbols + noise;
    end
end

Q = zeros(test_channel, 4, 4);
RR = zeros(test_channel, 4, 4);
for h=1:test_channel
    [Q_t,R_t] = qr(squeeze(Hmatrix(h, :, :)));
    Q(h, :, :) = Q_t';
    RR(h, :, :) = R_t;
end

z = zeros(test_channel, length(SNRdB), size(Tx_Symbols, 1), size(Tx_Symbols, 2));
for h = 1:test_channel
    for k = 1:length(SNRdB)
        z(h, k, :, :) = squeeze(Q(h, :, :))*squeeze(Rx_Symbols(h, k, :, :));
    end
end

%% ML detection
tic
BER_ML = zeros(size(SNRdB));

[set1, set2, set3, set4] = ndgrid(constellation, constellation, constellation, constellation);
allX = [set1(:), set2(:), set3(:), set4(:)].';

for h = 1:test_channel
    for k = 1:length(SNRdB)
        % Reshape received symbols and channel matrix
        Rx = squeeze(Rx_Symbols(h, k, :, :)); % [4, numSymbols/4]
        HH = squeeze(Hmatrix(h, :, :));        % [4, 4]

        % Precompute all candidate symbols for each column
        numCandidates = size(allX, 2);
        distances = zeros(numCandidates, size(Rx, 2));

        for col = 1:size(Rx, 2)
            x_candidates = allX; % [4, numCandidates]
            distances(:, col) = vecnorm(Rx(:, col) - HH * x_candidates, 2, 1); % [numCandidates, 1]
        end

        % Find minimum distances for each symbol block
        [~, min_indices] = min(distances, [], 1);
        x_ML = allX(:, min_indices); % Detected symbols

        % Map detected symbols to bit indices
        [~, indices] = ismember(x_ML, constellation);
        indices = reshape(indices, 1, []);
        
        % Decode bits
        detectedBits = de2bi(indices - 1, bitsPerSymbol, 'left-msb').';
        detectedBits = detectedBits(:)'; % Reshape to a 1D array

        % Compute bit errors
        numErrors = sum(bitData ~= detectedBits);
        BER_ML(k) = BER_ML(k) + (numErrors / length(bitData));
    end
end

BER_ML = BER_ML / test_channel;
toc

%% Breadth-First-Search (floating-point)
tic
BER_BF_float = zeros(size(SNRdB));

K = 8;
for h = 1:test_channel
    for k = 1:length(SNRdB)
        x_hat = zeros(4, numSymbols/4);
        for symbol = 1:numSymbols/4
            % 第 4 層
            level_4 = zeros(1, 8);
            for id = 1:8
                level_4(id) = abs(z(h, k, 4, symbol) - RR(h, 4, 4) * constellation(id))^2;
            end
            [min_values, indices] = mink(level_4, K);
            id_level_4 = indices;
    
            % 第 3 層
            level_3 = zeros(K, 8);
            for k1 = 1:K
                for id = 1:8
                    level_3(k1, id) = min_values(k1) + ...
                        abs(z(h, k, 3, symbol) - RR(h, 3, 4) * constellation(id_level_4(k1)) - RR(h, 3, 3) * constellation(id))^2;
                end 
            end
            [min_values, indices] = mink(reshape(level_3', 1, []), K);
            id_level_4 = id_level_4(floor((indices-1)/8)+1);
            id_level_3 = mod((indices-1), 8)+1;
            
            % 第 2 層
            level_2 = zeros(K, 8);
            for k2 = 1:K
                for id = 1:8
                    level_2(k2, id) = min_values(k2) + ...
                        abs(z(h, k, 2, symbol) - RR(h, 2, 4) * constellation(id_level_4(k2)) - ...
                        RR(h, 2, 3) * constellation(id_level_3(k2)) - RR(h, 2, 2) * constellation(id))^2;
                end
            end
            [min_values, indices] = mink(reshape(level_2', 1, []), K);
            id_level_4 = id_level_4(floor((indices-1)/8)+1);
            id_level_3 = id_level_3(floor((indices-1)/8)+1);
            id_level_2 = mod((indices-1), 8)+1;
    
            % 第 1 層
            level_1 = zeros(K, 8);
            for k3 = 1:K
                for id = 1:8
                    level_1(k3, id) = min_values(k3) + ...
                        abs(z(h, k, 1, symbol) - RR(h, 1, 4) * constellation(id_level_4(k3)) - ...
                        RR(h, 1, 3) * constellation(id_level_3(k3)) - ...
                        RR(h, 1, 2) * constellation(id_level_2(k3)) - ...
                        RR(h, 1, 1) * constellation(id))^2;
                end
            end
            [min_values, indices] = min(reshape(level_1', 1, []));
            x_hat(4, symbol) = id_level_4(floor((indices-1)/8)+1);
            x_hat(3, symbol) = id_level_3(floor((indices-1)/8)+1);
            x_hat(2, symbol) = id_level_2(floor((indices-1)/8)+1);
            x_hat(1, symbol) = mod((indices-1), 8)+1;
        end

        % Decode bits
        x_hat = reshape(x_hat, 1, []);
        detectedBits = zeros(1, numSymbols * bitsPerSymbol);
        for i = 1:length(x_hat)
            idx = x_hat(i) - 1;
            bits = de2bi(idx, bitsPerSymbol, 'left-msb');
            detectedBits((i-1)*bitsPerSymbol + (1:bitsPerSymbol)) = bits;
        end

        % Compute bit errors
        numErrors = sum(bitData ~= detectedBits);
        BER_BF_float(k) = BER_BF_float(k) + (numErrors / length(bitData));
    end
end

BER_BF_float = BER_BF_float / test_channel;
toc

%% Quantization
integer_bits = 4;
fractional_bits = 6;

R_Quan = zeros(test_channel, 4, 4);
z_Quan = zeros(test_channel, length(SNRdB), size(Tx_Symbols, 1), size(Tx_Symbols, 2));

for h = 1:test_channel
    [R_fixed_array, R_binary_array] = float_to_fixed(squeeze(RR(h, :, :)), integer_bits, fractional_bits);
    R_Quan(h, :, :) = binary_to_float(R_binary_array, integer_bits, fractional_bits);
    if (h == pattern_index)
        R_bin = R_binary_array;
    end
end

for h = 1:test_channel
    for k = 1:length(SNRdB)
        [z_fixed_array, z_binary_array] = float_to_fixed(squeeze(z(h, k, :, :)), integer_bits, fractional_bits);
        z_Quan(h, k, :, :) = binary_to_float(z_binary_array, integer_bits, fractional_bits);
        if (SNRdB(k) == 10 && h == pattern_index)
            z_SNR_bin = z_binary_array;
        end
    end
end


[fixed_array, binary_array] = float_to_fixed(constellation, integer_bits, fractional_bits);
binary_array(1, 1, 1) = {'1111010011'};
binary_array(1, 4, 1) = {'1111010011'};
binary_array(1, 1, 2) = {'1111010011'};
binary_array(1, 6, 2) = {'1111010011'};
constellation_fix = binary_to_float(binary_array, integer_bits, fractional_bits);


%% Breadth-First-Search (fix-point)
num_index = [11, 12, 13, 14, 15, 17, 18, 19, 20, 21, 22];

tic
BER_BF_fix = zeros(size(SNRdB));

K = 8;
num_cnt = 1;

check_ED = zeros(1, 11);

for h = 1:test_channel
    for k = 1:length(SNRdB)
        x_hat = zeros(4, numSymbols/4);
        for symbol = 1:numSymbols/4
            % 第 4 層
            level_4 = zeros(1, 8);
            for id = 1:8
                rr = floor( real(R_Quan(h, 4, 4)) * abs(real(constellation_fix(id))) * (2^fractional_bits) ) / (2^fractional_bits);
                ii = floor( imag(R_Quan(h, 4, 4)) * abs(imag(constellation_fix(id))) * (2^fractional_bits) ) / (2^fractional_bits);
                ri = floor( real(R_Quan(h, 4, 4)) * abs(imag(constellation_fix(id))) * (2^fractional_bits) ) / (2^fractional_bits);
                ir = floor( imag(R_Quan(h, 4, 4)) * abs(real(constellation_fix(id))) * (2^fractional_bits) ) / (2^fractional_bits);
                if (real(constellation_fix(id)) < 0)
                    rr = (-1) * rr;
                    ir = (-1) * ir;
                end
                if (imag(constellation_fix(id)) < 0)
                    ii = (-1) * ii;
                    ri = (-1) * ri;
                end
                h44 = (rr - ii) + 1j*(ri + ir);

                PED = (z_Quan(h, k, 4, symbol) - h44);
                e = abs(real(PED)) + abs(imag(PED));

                level_4(id) = e;
            end
            [min_values, indices] = mink(level_4, K);
            id_level_4 = indices;
    
            % 第 3 層
            level_3 = zeros(K, 8);
            for k1 = 1:K
                for id = 1:8
                    
                    rr = floor( real(R_Quan(h, 3, 4)) * abs(real(constellation_fix(id_level_4(k1)))) * (2^fractional_bits) ) / (2^fractional_bits);
                    ii = floor( imag(R_Quan(h, 3, 4)) * abs(imag(constellation_fix(id_level_4(k1)))) * (2^fractional_bits) ) / (2^fractional_bits);
                    ri = floor( real(R_Quan(h, 3, 4)) * abs(imag(constellation_fix(id_level_4(k1)))) * (2^fractional_bits) ) / (2^fractional_bits);
                    ir = floor( imag(R_Quan(h, 3, 4)) * abs(real(constellation_fix(id_level_4(k1)))) * (2^fractional_bits) ) / (2^fractional_bits);
                    if (real(constellation_fix(id_level_4(k1))) < 0)
                        rr = (-1) * rr;
                        ir = (-1) * ir;
                    end
                    if (imag(constellation_fix(id_level_4(k1))) < 0)
                        ii = (-1) * ii;
                        ri = (-1) * ri;
                    end
                    h34 = (rr - ii) + 1j*(ri + ir);

                    rr = floor( real(R_Quan(h, 3, 3)) * abs(real(constellation_fix(id))) * (2^fractional_bits) ) / (2^fractional_bits);
                    ii = floor( imag(R_Quan(h, 3, 3)) * abs(imag(constellation_fix(id))) * (2^fractional_bits) ) / (2^fractional_bits);
                    ri = floor( real(R_Quan(h, 3, 3)) * abs(imag(constellation_fix(id))) * (2^fractional_bits) ) / (2^fractional_bits);
                    ir = floor( imag(R_Quan(h, 3, 3)) * abs(real(constellation_fix(id))) * (2^fractional_bits) ) / (2^fractional_bits);
                    if (real(constellation_fix(id)) < 0)
                        rr = (-1) * rr;
                        ir = (-1) * ir;
                    end
                    if (imag(constellation_fix(id)) < 0)
                        ii = (-1) * ii;
                        ri = (-1) * ri;
                    end
                    h33 = (rr - ii) + 1j*(ri + ir);

                    PED = (z_Quan(h, k, 3, symbol) - h34 - h33);
                    e = abs(real(PED)) + abs(imag(PED));

                    level_3(k1, id) = min_values(k1) + e;
                end 
            end
            [min_values, indices] = mink(reshape(level_3', 1, []), K);
            id_level_4 = id_level_4(floor((indices-1)/8)+1);
            id_level_3 = mod((indices-1), 8)+1;
            
            % 第 2 層
            level_2 = zeros(K, 8);
            for k2 = 1:K
                for id = 1:8
                    rr = floor( real(R_Quan(h, 2, 4)) * abs(real(constellation_fix(id_level_4(k2)))) * (2^fractional_bits) ) / (2^fractional_bits);
                    ii = floor( imag(R_Quan(h, 2, 4)) * abs(imag(constellation_fix(id_level_4(k2)))) * (2^fractional_bits) ) / (2^fractional_bits);
                    ri = floor( real(R_Quan(h, 2, 4)) * abs(imag(constellation_fix(id_level_4(k2)))) * (2^fractional_bits) ) / (2^fractional_bits);
                    ir = floor( imag(R_Quan(h, 2, 4)) * abs(real(constellation_fix(id_level_4(k2)))) * (2^fractional_bits) ) / (2^fractional_bits);
                    if (real(constellation_fix(id_level_4(k2))) < 0)
                        rr = (-1) * rr;
                        ir = (-1) * ir;
                    end
                    if (imag(constellation_fix(id_level_4(k2))) < 0)
                        ii = (-1) * ii;
                        ri = (-1) * ri;
                    end
                    h24 = (rr - ii) + 1j*(ri + ir);

                    rr = floor( real(R_Quan(h, 2, 3)) * abs(real(constellation_fix(id_level_3(k2)))) * (2^fractional_bits) ) / (2^fractional_bits);
                    ii = floor( imag(R_Quan(h, 2, 3)) * abs(imag(constellation_fix(id_level_3(k2)))) * (2^fractional_bits) ) / (2^fractional_bits);
                    ri = floor( real(R_Quan(h, 2, 3)) * abs(imag(constellation_fix(id_level_3(k2)))) * (2^fractional_bits) ) / (2^fractional_bits);
                    ir = floor( imag(R_Quan(h, 2, 3)) * abs(real(constellation_fix(id_level_3(k2)))) * (2^fractional_bits) ) / (2^fractional_bits);
                    if (real(constellation_fix(id_level_3(k2))) < 0)
                        rr = (-1) * rr;
                        ir = (-1) * ir;
                    end
                    if (imag(constellation_fix(id_level_3(k2))) < 0)
                        ii = (-1) * ii;
                        ri = (-1) * ri;
                    end
                    h23 = (rr - ii) + 1j*(ri + ir);

                    rr = floor( real(R_Quan(h, 2, 2)) * abs(real(constellation_fix(id))) * (2^fractional_bits) ) / (2^fractional_bits);
                    ii = floor( imag(R_Quan(h, 2, 2)) * abs(imag(constellation_fix(id))) * (2^fractional_bits) ) / (2^fractional_bits);
                    ri = floor( real(R_Quan(h, 2, 2)) * abs(imag(constellation_fix(id))) * (2^fractional_bits) ) / (2^fractional_bits);
                    ir = floor( imag(R_Quan(h, 2, 2)) * abs(real(constellation_fix(id))) * (2^fractional_bits) ) / (2^fractional_bits);
                    if (real(constellation_fix(id)) < 0)
                        rr = (-1) * rr;
                        ir = (-1) * ir;
                    end
                    if (imag(constellation_fix(id)) < 0)
                        ii = (-1) * ii;
                        ri = (-1) * ri;
                    end
                    h22 = (rr - ii) + 1j*(ri + ir);

                    PED = (z_Quan(h, k, 2, symbol) - h24 - h23 - h22);
                    e = abs(real(PED)) + abs(imag(PED));

                    level_2(k2, id) = min_values(k2) + e;
                end
            end
            [min_values, indices] = mink(reshape(level_2', 1, []), K);
            id_level_4 = id_level_4(floor((indices-1)/8)+1);
            id_level_3 = id_level_3(floor((indices-1)/8)+1);
            id_level_2 = mod((indices-1), 8)+1;
    
            % 第 1 層
            level_1 = zeros(K, 8);
            for k3 = 1:K
                for id = 1:8
                    rr = floor( real(R_Quan(h, 1, 4)) * abs(real(constellation_fix(id_level_4(k3)))) * (2^fractional_bits) ) / (2^fractional_bits);
                    ii = floor( imag(R_Quan(h, 1, 4)) * abs(imag(constellation_fix(id_level_4(k3)))) * (2^fractional_bits) ) / (2^fractional_bits);
                    ri = floor( real(R_Quan(h, 1, 4)) * abs(imag(constellation_fix(id_level_4(k3)))) * (2^fractional_bits) ) / (2^fractional_bits);
                    ir = floor( imag(R_Quan(h, 1, 4)) * abs(real(constellation_fix(id_level_4(k3)))) * (2^fractional_bits) ) / (2^fractional_bits);
                    if (real(constellation_fix(id_level_4(k3))) < 0)
                        rr = (-1) * rr;
                        ir = (-1) * ir;
                    end
                    if (imag(constellation_fix(id_level_4(k3))) < 0)
                        ii = (-1) * ii;
                        ri = (-1) * ri;
                    end
                    h14 = (rr - ii) + 1j*(ri + ir);

                    rr = floor( real(R_Quan(h, 1, 3)) * abs(real(constellation_fix(id_level_3(k3)))) * (2^fractional_bits) ) / (2^fractional_bits);
                    ii = floor( imag(R_Quan(h, 1, 3)) * abs(imag(constellation_fix(id_level_3(k3)))) * (2^fractional_bits) ) / (2^fractional_bits);
                    ri = floor( real(R_Quan(h, 1, 3)) * abs(imag(constellation_fix(id_level_3(k3)))) * (2^fractional_bits) ) / (2^fractional_bits);
                    ir = floor( imag(R_Quan(h, 1, 3)) * abs(real(constellation_fix(id_level_3(k3)))) * (2^fractional_bits) ) / (2^fractional_bits);
                    if (real(constellation_fix(id_level_3(k3))) < 0)
                        rr = (-1) * rr;
                        ir = (-1) * ir;
                    end
                    if (imag(constellation_fix(id_level_3(k3))) < 0)
                        ii = (-1) * ii;
                        ri = (-1) * ri;
                    end
                    h13 = (rr - ii) + 1j*(ri + ir);

                    rr = floor( real(R_Quan(h, 1, 2)) * abs(real(constellation_fix(id_level_2(k3)))) * (2^fractional_bits) ) / (2^fractional_bits);
                    ii = floor( imag(R_Quan(h, 1, 2)) * abs(imag(constellation_fix(id_level_2(k3)))) * (2^fractional_bits) ) / (2^fractional_bits);
                    ri = floor( real(R_Quan(h, 1, 2)) * abs(imag(constellation_fix(id_level_2(k3)))) * (2^fractional_bits) ) / (2^fractional_bits);
                    ir = floor( imag(R_Quan(h, 1, 2)) * abs(real(constellation_fix(id_level_2(k3)))) * (2^fractional_bits) ) / (2^fractional_bits);
                    if (real(constellation_fix(id_level_2(k3))) < 0)
                        rr = (-1) * rr;
                        ir = (-1) * ir;
                    end
                    if (imag(constellation_fix(id_level_2(k3))) < 0)
                        ii = (-1) * ii;
                        ri = (-1) * ri;
                    end
                    h12 = (rr - ii) + 1j*(ri + ir);

                    rr = floor( real(R_Quan(h, 1, 1)) * abs(real(constellation_fix(id))) * (2^fractional_bits) ) / (2^fractional_bits);
                    ii = floor( imag(R_Quan(h, 1, 1)) * abs(imag(constellation_fix(id))) * (2^fractional_bits) ) / (2^fractional_bits);
                    ri = floor( real(R_Quan(h, 1, 1)) * abs(imag(constellation_fix(id))) * (2^fractional_bits) ) / (2^fractional_bits);
                    ir = floor( imag(R_Quan(h, 1, 1)) * abs(real(constellation_fix(id))) * (2^fractional_bits) ) / (2^fractional_bits);
                    if (real(constellation_fix(id)) < 0)
                        rr = (-1) * rr;
                        ir = (-1) * ir;
                    end
                    if (imag(constellation_fix(id)) < 0)
                        ii = (-1) * ii;
                        ri = (-1) * ri;
                    end
                    h11 = (rr - ii) + 1j*(ri + ir);

                    
                    PED = (z_Quan(h, k, 1, symbol) - h14 - h13 - h12 - h11);
                    e = abs(real(PED)) + abs(imag(PED));
                    level_1(k3, id) = min_values(k3) + e;
                end
            end
            [min_values, indices] = min(reshape(level_1', 1, []));
            x_hat(4, symbol) = id_level_4(floor((indices-1)/8)+1);
            x_hat(3, symbol) = id_level_3(floor((indices-1)/8)+1);
            x_hat(2, symbol) = id_level_2(floor((indices-1)/8)+1);
            x_hat(1, symbol) = mod((indices-1), 8)+1;
            
            if (num_cnt < 12)
                if (SNRdB(k) == 10 && h == pattern_index && (symbol == num_index(num_cnt)))
                    check_ED(num_cnt) = min_values;
                    num_cnt = num_cnt + 1;
                end
            end
        end

        if (SNRdB(k) == 10 && h == pattern_index)
            x_hat_SNR = x_hat;
        end

        % Decode bits
        x_hat = reshape(x_hat, 1, []);
        detectedBits = zeros(1, numSymbols * bitsPerSymbol);
        for i = 1:length(x_hat)
            idx = x_hat(i) - 1;
            bits = de2bi(idx, bitsPerSymbol, 'left-msb');
            detectedBits((i-1)*bitsPerSymbol + (1:bitsPerSymbol)) = bits;
        end

        % Compute bit errors
        numErrors = sum(bitData ~= detectedBits);
        BER_BF_fix(k) = BER_BF_fix(k) + (numErrors / length(bitData));
    end
end

BER_BF_fix = BER_BF_fix / test_channel;
toc

%%
target_BER = 10^-4;
ML_left = 0; ML_right = 0; BF_fix_left = 0; BF_fix_right = 0;

% Find the indices for interpolation
% Find the indices for interpolation
for s = 1:length(SNRdB)-1
    if ((BER_ML(s) > target_BER) && (BER_ML(s+1) <= target_BER))
        ML_left = s;
        ML_right = s+1;
        break;
    end
end
for s = 1:length(SNRdB)-1
    if ((BER_BF_fix(s) > target_BER) && (BER_BF_fix(s+1) <= target_BER))
        BF_fix_left = s;
        BF_fix_right = s+1;
        break;
    end
end

% Perform interpolation
ML_target_SNR = ((log10(BER_ML(ML_right)) - log10(target_BER)) * (SNRdB(ML_left) - SNRdB(ML_right)) / ...
                 (log10(BER_ML(ML_right)) - log10(BER_ML(ML_left)))) + SNRdB(ML_right);

BF_fix_target_SNR = ((log10(BER_BF_fix(BF_fix_right)) - log10(target_BER)) * (SNRdB(BF_fix_left) - SNRdB(BF_fix_right)) / ...
                 (log10(BER_BF_fix(BF_fix_right)) - log10(BER_BF_fix(BF_fix_left)))) + SNRdB(BF_fix_right);

degradation = BF_fix_target_SNR - ML_target_SNR;

% Plot BER vs SNR with log-scaled x-axis
figure;
semilogy(SNRdB, BER_ML, 'b-o', 'DisplayName', 'ML'); % Convert SNRdB to linear scale for log axis
hold on;
semilogy(SNRdB, BER_BF_float, 'r-o', 'DisplayName', 'BF-float');
semilogy(SNRdB, BER_BF_fix, 'g-o', 'DisplayName', 'BF-fix');

semilogy(ML_target_SNR, target_BER, 'b-o', 'MarkerFaceColor','r');
text(ML_target_SNR-5, target_BER+0.06, sprintf('ML-SNR: %f', ML_target_SNR), 'FontSize',13);
semilogy(BF_fix_target_SNR, target_BER, 'g-o', 'MarkerFaceColor','r');
text(BF_fix_target_SNR-7, target_BER+0.02, sprintf('Fixpoint-BFS-SNR: %f', BF_fix_target_SNR), 'FontSize',13);
text(BF_fix_target_SNR-5.3, target_BER+0.008, sprintf('degradation: %f', degradation), 'FontSize',13);
set(gca,'FontSize',13);
hold off;

% Add legend and labels
legend('ML', 'BF-float', 'BF-fix');
xticks(min(SNRdB):1:max(SNRdB));
xlabel('SNR (linear scale)');
ylim([5*10^(-5) 1]);
ylabel('Bit Error Rate (BER)');
grid on;

%% Write Pattern (註解拿掉就可以用了)

% fid = fopen('channel.dat','wt');
% for row = 1:4
%     for col = 1:4
%         fprintf(fid, '%s', char(R_bin(row, col, 1)));
%         fprintf(fid, '%s', char(R_bin(row, col, 2)));
%         fprintf(fid, '\n');
%     end
% end
% fclose(fid);
% 
% fid = fopen('z.dat','wt');
% for num = 1:11
%     for ele = 1:4
%         fprintf(fid, '%s', char(z_SNR_bin(ele, num_index(num), 1)));
%         fprintf(fid, '%s', char(z_SNR_bin(ele, num_index(num), 2)));
%         fprintf(fid, '\n');
%     end
% end
% fclose(fid);
% 
% fid = fopen('golden.dat','wt');
% for num = 1:11
%     for ele = 1:4
%         fprintf(fid, '%s', char(dec2bin(x_hat_SNR(ele, num_index(num))-1, 3)));
%     end
%     fprintf(fid, '\n');
% end
% fclose(fid);
% 
% [z_fixed_array, check_ED_bin] = float_to_fixed(check_ED, integer_bits, fractional_bits);
% fid = fopen('distance.dat','wt');
% for num = 1:11
%     fprintf(fid, '%s', char(check_ED_bin(num)));
%     fprintf(fid, '\n');
% end
% fclose(fid);

%%
function [fixed_array, binary_array] = float_to_fixed(np_array, integer_bits, fractional_bits)
    % 計算總位元數與縮放因子
    total_bits = integer_bits + fractional_bits;
    scaling_factor = 2 ^ fractional_bits;

    % 轉換為定點數 (放大並取整)
    fixed_array = floor(np_array * scaling_factor);

    % 定義二進位轉換的函數
    function binary_str = to_binary(value)
        max_value = (2^(total_bits - 1)) - 1; % 最大值
        min_value = -(2^(total_bits - 1));    % 最小值

        % 限制範圍
        value = min(max(value, min_value), max_value);

        % 計算二進位表示
        if value >= 0
            binary_str = dec2bin(value, total_bits);
        else
            binary_str = dec2bin((2^total_bits) + value, total_bits);
        end
    end

    % 初始化二進位陣列
    [rows, cols] = size(np_array);
    binary_array = cell(rows, cols, 2);

    % 對每個元素進行處理
    for i = 1:rows
        for j = 1:cols
            real_part = real(fixed_array(i, j));
            imag_part = imag(fixed_array(i, j));
            binary_array{i, j, 1} = to_binary(real_part); % 實部
            binary_array{i, j, 2} = to_binary(imag_part); % 虛部
        end
    end
end

function float_array = binary_to_float(binary_array, integer_bits, fractional_bits)
    % 總位元數
    total_bits = integer_bits + fractional_bits;

    % 初始化浮點數陣列
    [rows, cols, ~] = size(binary_array);
    float_array = zeros(rows, cols);

    % 遍歷每個元素並解碼
    for i = 1:rows
        for j = 1:cols
            % 解碼實部
            real_binary = binary_array{i, j, 1};
            real_value = binary_to_decimal(real_binary, total_bits);

            % 解碼虛部
            imag_binary = binary_array{i, j, 2};
            imag_value = binary_to_decimal(imag_binary, total_bits);

            % 恢復縮放的浮點數
            float_array(i, j) = real_value / (2 ^ fractional_bits) + ...
                                1i * (imag_value / (2 ^ fractional_bits));
        end
    end
end

function decimal_value = binary_to_decimal(binary_str, total_bits)
    % 轉換二進位字串為整數
    value = bin2dec(binary_str);
    
    % 確認是否為負數
    if value >= 2^(total_bits - 1)
        decimal_value = value - 2^total_bits; % 處理補碼負數
    else
        decimal_value = value; % 正數保持不變
    end
end