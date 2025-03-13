import cv2

 

cap = cv2.VideoCapture(0)

 

while cap.isOpened():

    ret, frame = cap.read()

    frame = cv2.flip(frame, 1)
    print(frame)

    

    if ret:

        cv2.imshow('frame', frame)

 

        if cv2.waitKey(1) & 0xFF == ord('q'):

            # 키보드에서 'q'를 입력하면 종료

            break

    else:

        break

 

cap.release()

cv2.destroyAllWindows()