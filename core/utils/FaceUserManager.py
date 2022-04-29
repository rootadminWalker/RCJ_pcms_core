import os
import pickle
import pprint
from typing import List

import numpy as np
import pandas as pd

from RCJ_pcms_core.core.Dtypes.FaceProcess import FaceUser


class FaceUserManager:
    def __init__(self, database: str):
        self.database: str = database
        self.users: List[FaceUser] = []

        # Load database base on format of database
        if database.endswith('pickle'):
            self._load_pickle_database(database)
        elif os.path.isdir(database):
            self._load_directory_csv_database(database)

    def __repr__(self):
        return f'FaceUserManager at {hex(id(self))} with users:\n{pprint.pformat(self.users)}'

    def _load_pickle_database(self, database):
        # Read the pickle database file
        unconverted_users = pickle.load(open(database, 'rb'))
        # Pickle format:
        # {<username1>: <description1>, <username2>: <description2>}
        for username, description in unconverted_users.items():
            # Get users from pickle and convert to FaceUser
            self.users.append(FaceUser(username=username, descriptor=np.array(description)))

    def _load_directory_csv_database(self, database):
        for user_csv in os.listdir(database):
            # Read csv userdata using pandas
            csv_userdata: pd.DataFrame = pd.read_csv(f'{database}/{user_csv}.csv')

            # CSV format:
            # Column 1: <username>
            # Data of Column 1: <description>
            username = csv_userdata.columns[0]
            description = np.array(csv_userdata[username])

            # Convert data into FaceUser
            self.users.append(FaceUser(username=username, descriptor=description))

    def sign_in(self, face: FaceUser) -> FaceUser:
        for seen_user in self.users:
            if face.compare_with_another_face(seen_user) < 0.4:
                return seen_user
        else:
            return FaceUser(username='0', descriptor=[])