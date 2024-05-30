FROM alpine:latest AS builder
RUN apk add --no-cache cmake make git boost-dev=1.82.0-r5 g++
WORKDIR /app
COPY . .
WORKDIR /app/build
RUN cmake .. && make

FROM alpine:latest
# Libs
COPY --from=builder /usr/lib/libstdc++.so.6 /usr/lib/libstdc++.so.6
COPY --from=builder /usr/lib/libgcc_s.so.1 /usr/lib/libgcc_s.so.1
COPY --from=builder /usr/lib/libboost_program_options.so.1.82.0 /usr/lib/libboost_program_options.so.1.82.0
COPY --from=builder /usr/lib/libatomic.so.1 /usr/lib/libatomic.so.1
COPY --from=builder /app/build/iso22133/libISO22133.so /app/build/iso22133/libISO22133.so
COPY --from=builder /app/build/libISO_object.so /app/build/iso22133/libISO_object.so
# App
COPY --from=builder /app/build/ISO_objectDemo /app/build/ISO_objectDemo

ENTRYPOINT ["/app/build/ISO_objectDemo"]