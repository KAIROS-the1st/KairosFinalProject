import React, {useEffect} from 'react';
import { StyleSheet, View, Alert, TouchableOpacity, Text } from 'react-native';
import axios from 'axios';
import SplashScreen from 'react-native-splash-screen'

const App = () => {
  useEffect(() => {
    if (Platform.OS === 'android')  SplashScreen.hide();
  }, [])

  const handleYellowButtonPress = async () => {
    try {
      const response = await axios.post('http://172.30.1.22:5000/send_message', { message: 'start' });
      Alert.alert('Success', 'Message sent successfully!');
    } catch (error) {
      Alert.alert('Error', 'Failed to send message.');
    }
  };

  const handleBlueButtonPress = async () => {
    try {
      const response = await axios.post('http://172.30.1.34:5000/plc', { message: '4' });
      Alert.alert('Success', 'Message sent successfully!');
    } catch (error) {
      Alert.alert('Error', 'Failed to send message.');
    }
  };

  return (
    <View style={styles.container}>
      <TouchableOpacity style={styles.yellowButton} onPress={handleYellowButtonPress}>
        <Text style={styles.yellowButtonText}>Tap to Call</Text>
      </TouchableOpacity>
      <TouchableOpacity style={styles.blueButton} onPress={handleBlueButtonPress}>
        <Text style={styles.blueButtonText}>Send 4</Text>
      </TouchableOpacity>
    </View>
  );
};

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: '#FFFFFF',
  },
  yellowButton: {
    flex: 9,
    backgroundColor: 'yellow',
    justifyContent: 'center',
    alignItems: 'center',
  },
  blueButton: {
    flex: 1,
    backgroundColor: 'blue',
    justifyContent: 'center',
    alignItems: 'center',
  },
  yellowButtonText: {
    color: 'black',
    fontSize: 48,
  },
  blueButtonText: {
      color: 'white',
      fontSize: 18,
    },
});

export default App;


